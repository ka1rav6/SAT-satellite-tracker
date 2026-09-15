// gui/dashboard.cpp — see dashboard.hpp for what this is for.

#include "gui/dashboard.hpp"

#include "sat/version.hpp"
#include "scenario/schema.hpp"

#include "imgui.h"
#include "imgui_internal.h"      // DockBuilder — see build_default_layout()
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sat::gui {
namespace {

// A small palette, used consistently so a viewer learns what each colour means
// within the first few seconds rather than having to read a legend.
constexpr ImVec4 kTruthCol    {0.25f, 0.95f, 0.40f, 1.00f};   // green  = truth
constexpr ImVec4 kDetectCol   {1.00f, 0.55f, 0.10f, 1.00f};   // orange = detection
constexpr ImVec4 kBoreCol     {0.35f, 0.70f, 1.00f, 1.00f};   // blue   = where we point
constexpr ImVec4 kBudgetCol   {0.95f, 0.30f, 0.30f, 1.00f};   // red    = the spec limit
constexpr ImVec4 kMutedCol    {0.60f, 0.60f, 0.65f, 1.00f};

ImU32 col(const ImVec4& c) { return ImGui::ColorConvertFloat4ToU32(c); }

void glfw_error(int code, const char* desc) {
    std::fprintf(stderr, "glfw error %d: %s\n", code, desc);
}

/// A label plus a value, coloured by whether it meets its requirement. Used for
/// every compliance figure so pass and fail are legible at a glance from the
/// back of a room, which is what §14.1's demo actually needs.
void metric_row(const char* label, const char* fmt, double value,
                bool ok, const char* requirement) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();
    ImGui::TextColored(ok ? ImVec4{0.4f, 0.9f, 0.4f, 1.0f}
                          : ImVec4{1.0f, 0.45f, 0.35f, 1.0f}, fmt, value);
    ImGui::TableNextColumn();
    ImGui::TextColored(kMutedCol, "%s", requirement);
}

}  // namespace

namespace {
/// Arrange the panels once, on first run.
///
/// Without this, every panel is a free-floating window stacked at the origin,
/// which is what ImGui does by default and which looks exactly as broken as it
/// sounds. A demo has no time for someone to drag seven windows into place, and
/// §14.1 gives the whole presentation fifteen minutes.
///
/// The proportions follow what a viewer actually looks at: the camera view is
/// the centre of attention, the two graded error traces are stacked down the
/// right where they can be watched while something is dialled on the left, and
/// the controls sit on the left under the hand that is about to use them.
void build_default_layout(ImGuiID dockspace, DockIds& out) {
    // The flags here MUST match the ones the DockSpace call uses. If they
    // differ, ImGui decides the node is stale and rebuilds it from scratch on
    // the next frame, silently discarding this entire layout — which is exactly
    // what happened when AddNode was given only _DockSpace while the dockspace
    // was created with _PassthruCentralNode as well.
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->WorkSize);

    // ---------------------------------------------------------------------
    // BOTH sides of every split are captured.
    //
    // DockBuilderSplitNode turns the node it is given into a PARENT of two new
    // children and returns the one on the requested side. The original ID then
    // names the parent, not the remaining half — so docking into it afterwards
    // is ambiguous, and ImGui resolves the ambiguity by piling the windows into
    // one tab bar. That is exactly what happened when these splits discarded
    // their second output: four unrelated panels ended up tabbed together.
    // ---------------------------------------------------------------------
    ImGuiID centre = dockspace;
    ImGuiID left = 0, right = 0;
    ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left,  0.21f, &left,  &centre);
    ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.34f, &right, &centre);

    ImGuiID left_top = 0, left_bottom = 0;
    ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.42f, &left_bottom, &left_top);

    ImGuiID centre_top = 0, centre_bottom = 0;
    ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.40f, &centre_bottom, &centre_top);

    ImGuiID right_top = 0, right_rest = 0;
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.68f, &right_rest, &right_top);
    ImGuiID right_mid = 0, right_bottom = 0;
    ImGui::DockBuilderSplitNode(right_rest, ImGuiDir_Down, 0.52f, &right_bottom, &right_mid);

    out.left          = left_top;
    out.left_bottom   = left_bottom;
    out.centre        = centre_top;
    out.centre_bottom = centre_bottom;
    out.right         = right_top;
    out.right_mid     = right_mid;
    out.right_bottom  = right_bottom;

    ImGui::DockBuilderFinish(dockspace);

    if (std::getenv("SAT_GUI_DEBUG")) {
        std::fprintf(stderr,
            "[layout] dockspace=%u work=%.0fx%.0f  nodes: L=%u/%u C=%u/%u R=%u/%u/%u\n",
            dockspace, ImGui::GetMainViewport()->WorkSize.x,
            ImGui::GetMainViewport()->WorkSize.y,
            out.left, out.left_bottom, out.centre, out.centre_bottom,
            out.right, out.right_mid, out.right_bottom);
    }
}

}  // namespace

// ===========================================================================
// Simulation stepping
// ===========================================================================

int Dashboard::dock_cond() const noexcept {
    return redock_frames_ > 0 ? ImGuiCond_Always : ImGuiCond_Once;
}

void Dashboard::rebuild(const Scenario& sc) {
    scenario_ = sc;

    // Clutter is handled here rather than by a preset button, because it is a
    // property of the WORLD and changing it needs a rebuild. §9.1 deliberately
    // makes some clutter sources BRIGHTER than the beacon — which is why the
    // brightest-pixel detector locks onto one within a few frames (CP 4.11) —
    // so the dashboard opens with it off and the slider below turns it back on.
    scenario_.static_sources = clutter_sources_;
    scenario_.decoy_beacons  = decoy_beacons_;

    // See the comment on beacon_in_view_: acquisition is Stage 13, and until
    // then a randomly placed beacon is off-screen 92% of the time.
    if (beacon_in_view_ && !scenario_.targets.empty()) {
        TargetSpec& t = scenario_.targets[0];
        t.random_initial = false;
        // Offset from the camera's start by about a quarter of the frame, so
        // the loop has a real error to remove and the convergence is visible
        // rather than instantaneous.
        t.initial_px[0] = scenario_.initial_pos_px[0] + scenario_.resolution[0] * 0.25;
        t.initial_px[1] = scenario_.initial_pos_px[1] + scenario_.resolution[1] * 0.20;

        // A motion stack containing `constant` is expressing an ABSOLUTE
        // position (world_builder.cpp), so it would override the placement
        // above. Re-centre that component instead of fighting it.
        for (MotionSpec& m : t.motion) {
            if (m.kind == "constant") {
                m.offset_px[0] = t.initial_px[0];
                m.offset_px[1] = t.initial_px[1];
            }
        }
    }

    pipeline_.build_from_scenario(scenario_);

    // Open on the CLEAN preset rather than on the scenario's full damage.
    //
    // Not to flatter the system - the scenario is loaded exactly as written and
    // one click restores it - but because the current detector is the Stage 1
    // straw man, and at row 21's 10% impulse noise it fails outright (CP 4.11).
    // Opening on a failure would show nothing about the parts that work, and
    // the far more useful demonstration is to start clean and dial damage up
    // until it breaks, which is also §14.1's running order.
    if (start_clean_) {
        SensorChain& c = pipeline_.source().sensor();
        c.set_atmosphere(Atmosphere::Clear);
        c.noise().gaussian_sigma  = 0.0;
        c.noise().salt_pepper_p   = 0.0;
        c.noise().poisson_enabled = false;
        c.set_defects_enabled(false);
    }

    centroid_image_.clear();
    centroid_screen_.clear();
    tracking_.clear();
    saturation_.clear();
    truth_path_x_.clear(); truth_path_y_.clear();
    det_path_x_.clear();   det_path_y_.clear();

    frames_ = frames_detected_ = frames_in_fov_ = false_alarms_ = 0;
    worst_tracking_ = 0.0;
    finished_ = false;

    char buf[256];
    std::snprintf(buf, sizeof(buf), "loaded '%s'  %dx%d px screen, %dx%d camera, %.0f Hz",
                  scenario_.name.c_str(), scenario_.canvas_px[0], scenario_.canvas_px[1],
                  scenario_.resolution[0], scenario_.resolution[1],
                  static_cast<double>(scenario_.camera_hz));
    status_ = buf;
}

void Dashboard::step_simulation() {
    if (finished_) return;
    if (!pipeline_.step()) {
        finished_ = true;
        running_  = false;
        status_   = "run complete — press Reset to run it again";
        return;
    }

    const FrameRecord& r = pipeline_.last();
    ++frames_;
    if (r.detected)     ++frames_detected_;
    if (r.truth_in_fov) ++frames_in_fov_;
    if (r.false_alarm)  ++false_alarms_;
    worst_tracking_ = std::max(worst_tracking_, r.tracking_error_px);

    tracking_.push(r.time_s, r.tracking_error_px);
    saturation_.push(r.time_s, pipeline_.gimbal().saturation_frac());
    if (r.centroid_error_valid) {
        centroid_image_.push(r.time_s, r.centroid_error_px);
        centroid_screen_.push(r.time_s, r.centroid_error_screen_px);
    }

    // Subsample the paths: a 120 s run is 3600 frames, far more than a
    // 400-pixel-wide overview panel can distinguish.
    if (frames_ % 3 == 0) {
        if (r.truth_valid) {
            truth_path_x_.push_back(r.truth_screen.x);
            truth_path_y_.push_back(r.truth_screen.y);
        }
        if (r.detected) {
            det_path_x_.push_back(r.detection_screen.x);
            det_path_y_.push_back(r.detection_screen.y);
        }
        constexpr size_t kMaxPath = 3000;
        if (truth_path_x_.size() > kMaxPath) {
            truth_path_x_.erase(truth_path_x_.begin());
            truth_path_y_.erase(truth_path_y_.begin());
        }
        if (det_path_x_.size() > kMaxPath) {
            det_path_x_.erase(det_path_x_.begin());
            det_path_y_.erase(det_path_y_.begin());
        }
    }
}

// ===========================================================================
// Panels
// ===========================================================================

void Dashboard::draw_menu_bar() {
    if (!ImGui::BeginMainMenuBar()) return;

    ImGui::TextColored(kMutedCol, "SAT %s+%s", SAT_VERSION, SAT_GIT_HASH);
    ImGui::Separator();

    if (ImGui::BeginMenu("Scenario")) {
        // The shipped scenarios, loadable live. CP 15.3 requires a known-good
        // fallback preloaded for the demo; baseline is it.
        for (const char* name : {"baseline.toml", "fog_figure8.toml",
                                 "maxnoise_random.toml"}) {
            if (ImGui::MenuItem(name)) {
                auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/" + name);
                if (r) {
                    rebuild(*r);
                } else {
                    status_ = "load failed: " + r.error();
                }
            }
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        // The panels are draggable, so there has to be a way back.
        if (ImGui::MenuItem("Reset layout")) {
            layout_built_  = false;
            stable_frames_ = 8;      // rebuild on the next frame
        }
        ImGui::EndMenu();
    }
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "%s", status_.c_str());
    ImGui::EndMainMenuBar();
}

void Dashboard::draw_controls() {
    if (dock_.left) ImGui::SetNextWindowDockID(dock_.left, dock_cond());
    ImGui::Begin("Run control");

    // --- transport ---------------------------------------------------------
    if (ImGui::Button(running_ ? "Pause" : "Play", ImVec2(70, 0))) running_ = !running_;
    ImGui::SameLine();
    if (ImGui::Button("Step", ImVec2(60, 0))) { step_once_ = true; running_ = false; }
    ImGui::SameLine();
    if (ImGui::Button("Reset", ImVec2(60, 0))) rebuild(scenario_);

    ImGui::SliderInt("sim frames / draw", &steps_per_draw_, 1, 30);

    if (ImGui::Checkbox("beacon in view at t=0", &beacon_in_view_)) {
        rebuild(scenario_);
    }
    ImGui::SetItemTooltip(
        "Specification row 11 places the beacon at a RANDOM position, and the\n"
        "headless runs honour that. But the camera sees 7.68%% of the screen,\n"
        "so a random beacon starts off-screen 92%% of the time — and finding it\n"
        "is the search problem, which is Stage 13 and not built yet.\n\n"
        "Turn this off to see exactly why Stage 13 is needed.");
    ImGui::TextColored(kMutedCol,
        "frame %lld of %lld   t = %.2f s",
        static_cast<long long>(frames_),
        static_cast<long long>(scenario_.duration_s * scenario_.camera_hz),
        pipeline_.last().time_s);

    ImGui::Separator();

    // -----------------------------------------------------------------------
    // THE DEMO CONTROL. Design §14.1 (5:00-6:30): turn the loop off live and
    // watch the error trace blow up. §12 calls being able to do this "worth
    // more than any table", and it is also CP 1.8's acceptance criterion (b) —
    // the thing that proves the camera follows because our controller told it
    // to, rather than because the simulator centres it.
    // -----------------------------------------------------------------------
    bool control_on = pipeline_.control_enabled();
    ImGui::PushStyleColor(ImGuiCol_Text, control_on ? ImVec4{0.4f, 0.9f, 0.4f, 1.0f}
                                                    : ImVec4{1.0f, 0.45f, 0.35f, 1.0f});
    if (ImGui::Checkbox("closed loop (INV-2)", &control_on)) {
        pipeline_.set_control_enabled(control_on);
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip(
        "Turn this off and the controller's output stops reaching the mount.\n"
        "The camera should immediately stop following (CP 1.8 acceptance (b)).");

    // --- damage, dialled live (§14.1, 3:00-5:00) ---------------------------
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Damage — specification rows 21-25");

    SensorChain& chain = pipeline_.source().sensor();

    // Presets, because §14.1's demo dials damage up in stages and hunting for
    // four sliders mid-presentation is not something anyone should have to do.
    if (ImGui::Button("Clean")) {
        chain.set_atmosphere(Atmosphere::Clear);
        chain.noise().gaussian_sigma  = 0.0;
        chain.noise().salt_pepper_p   = 0.0;
        chain.noise().poisson_enabled = false;
        chain.set_defects_enabled(false);
    }
    ImGui::SetItemTooltip("No damage at all. The loop should track to a few pixels.");
    ImGui::SameLine();
    if (ImGui::Button("Sensor noise")) {
        chain.set_atmosphere(Atmosphere::Clear);
        chain.noise().gaussian_sigma  = 20.0;   // row 22, at the cap
        chain.noise().salt_pepper_p   = 0.0;
        chain.noise().poisson_enabled = true;   // row 21
        chain.set_defects_enabled(false);
    }
    ImGui::SetItemTooltip(
        "Shot and read noise at the specification maximum. Both are additive\n"
        "and roughly symmetric, so a centre-of-mass estimator averages most of\n"
        "them away and the loop still holds.");
    ImGui::SameLine();
    if (ImGui::Button("Full spec")) {
        chain.set_atmosphere(scenario_.atmosphere);
        chain.noise().gaussian_sigma  = scenario_.gaussian_sigma;
        chain.noise().salt_pepper_p   = scenario_.salt_pepper;
        chain.noise().poisson_enabled = scenario_.noise_poisson;
        chain.set_defects_enabled(true);
    }
    // SetItemTooltip is PRINTF-STYLE. "10%" was being read as the conversion
    // "% i", so this pulled an int argument that was never passed — undefined
    // behaviour in a tooltip nobody would think to distrust. Escaped as "%%".
    ImGui::SetItemTooltip(
        "Everything the scenario asks for, including row 21's 10%% impulse\n"
        "noise. The straw-man detector does NOT survive this - watch the\n"
        "tracking trace leave the plot. That failure is CP 4.11.");

    bool damage = chain.enabled();
    if (ImGui::Checkbox("damage chain enabled", &damage)) chain.set_enabled(damage);

    int atm = static_cast<int>(chain.atmosphere());
    if (ImGui::Combo("atmosphere (row 24)", &atm,
                     "clear\0haze\0rain\0fog\0low light\0")) {
        chain.set_atmosphere(static_cast<Atmosphere>(atm));
    }

    NoiseParams& n = chain.noise();
    float sigma = static_cast<float>(n.gaussian_sigma);
    if (ImGui::SliderFloat("read noise sigma (row 22)", &sigma, 0.0f, 20.0f, "%.1f")) {
        n.gaussian_sigma = sigma;
    }
    float sp = static_cast<float>(n.salt_pepper_p);
    if (ImGui::SliderFloat("salt & pepper (row 21)", &sp, 0.0f, 0.30f, "%.3f")) {
        n.salt_pepper_p = sp;
    }
    ImGui::SetItemTooltip(
        "The specification asks for ~0.10. At that level the brightest-pixel\n"
        "detector fails outright: 30,720 impulse pixels against a 100-pixel\n"
        "beacon, every one of them brighter. This is what the Stage 5 median\n"
        "filter exists to remove.");
    ImGui::Checkbox("shot noise (row 21)", &n.poisson_enabled);

    bool defects = chain.defects_enabled();
    if (ImGui::Checkbox("hot / dead pixels", &defects)) chain.set_defects_enabled(defects);
    ImGui::SetItemTooltip(
        "40 pixels stuck at 255 and 10 stuck at 0, in fixed positions.\n"
        "These alone defeat a brightest-pixel detector - a stuck 255 beats a\n"
        "120-level beacon with no noise present at all. They are isolated\n"
        "single pixels, which is exactly what a median filter removes.");

    // Clutter needs a world rebuild, so it sits apart from the live sliders.
    ImGui::Spacing();
    ImGui::TextColored(kMutedCol, "Clutter (design §9.1) — rebuilds the world");
    bool rebuild_needed = false;
    rebuild_needed |= ImGui::SliderInt("static sources", &clutter_sources_, 0, 400);
    rebuild_needed |= ImGui::SliderInt("decoy beacons",  &decoy_beacons_,   0, 4);
    if (rebuild_needed && !ImGui::IsItemActive()) rebuild(scenario_);
    ImGui::SetItemTooltip(
        "§9.1 calls 50-500 static sources \"mandatory for credibility\", and\n"
        "deliberately makes some BRIGHTER than the beacon. Drag this up and\n"
        "watch the brightest-pixel detector lock onto one of them instead -\n"
        "that is CP 4.11.");

    ImGui::Separator();
    ImGui::TextColored(ImVec4{1.0f, 0.80f, 0.35f, 1.0f},
                       "Detector: brightest pixel - a deliberate straw man");
    ImGui::TextWrapped(
        "Stage 5 replaces this with the real pipeline: a 3x3 median, a van Herk "
        "top-hat, integer summed-area tables, a multi-scale matched filter and "
        "CFAR. Until then, press \"Full spec\" above to watch this detector fail "
        "at row 21's 10%% impulse noise - 30,720 corrupted pixels against a "
        "100-pixel beacon, every one of them brighter. That is CP 4.11, and it "
        "is the measured justification for building the median filter.");

    ImGui::End();
}

void Dashboard::draw_camera_view() {
    if (dock_.centre) ImGui::SetNextWindowDockID(dock_.centre, dock_cond());
    ImGui::Begin("Camera view");

    const SimSnapshot& snap = pipeline_.snapshots().read_slot();
    if (snap.preview_w > 0) {
        camera_tex_.upload_grey(snap.preview, snap.preview_w, snap.preview_h);
    }

    if (!camera_tex_.valid()) {
        ImGui::TextColored(kMutedCol, "no frame yet");
        ImGui::End();
        return;
    }

    // Fit the image to the panel while preserving aspect, so a resized window
    // never distorts a scientific image.
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  scale = std::min(avail.x / camera_tex_.width(),
                                  std::max(avail.y - 24.0f, 64.0f) / camera_tex_.height());
    const ImVec2 size{camera_tex_.width() * scale, camera_tex_.height() * scale};
    const ImVec2 origin = ImGui::GetCursorScreenPos();

    ImGui::Image(static_cast<ImTextureID>(camera_tex_.id()), size);

    // --- overlays ----------------------------------------------------------
    // Drawn with ImGui's draw list rather than in GL: it is the same primitives
    // either way, and this needs no shader, no vertex buffer and no state to
    // restore.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    auto to_screen = [&](Pixel2 p) {
        return ImVec2{origin.x + static_cast<float>(p.x) * scale,
                      origin.y + static_cast<float>(p.y) * scale};
    };

    // Boresight crosshair — the centre of the sensor, by definition.
    const ImVec2 c{origin.x + size.x * 0.5f, origin.y + size.y * 0.5f};
    dl->AddLine({c.x - 14, c.y}, {c.x - 4, c.y}, col(kBoreCol), 1.5f);
    dl->AddLine({c.x + 4, c.y}, {c.x + 14, c.y}, col(kBoreCol), 1.5f);
    dl->AddLine({c.x, c.y - 14}, {c.x, c.y - 4}, col(kBoreCol), 1.5f);
    dl->AddLine({c.x, c.y + 4}, {c.x, c.y + 14}, col(kBoreCol), 1.5f);

    const FrameRecord& r = pipeline_.last();

    // Truth, in green. Metrics-only information — the tracker never sees it,
    // and it is drawn here purely so a viewer can judge the tracker rather than
    // take its word.
    if (r.truth_valid && r.truth_in_fov) {
        const Pixel2 img = screen_to_image(scenario_.camera_geometry(),
                                           scenario_.screen_geometry(),
                                           r.truth_screen, r.boresight_true);
        const ImVec2 p = to_screen(img);
        dl->AddCircle(p, 11.0f, col(kTruthCol), 0, 1.8f);
        dl->AddLine({p.x - 16, p.y}, {p.x - 12, p.y}, col(kTruthCol), 1.5f);
        dl->AddLine({p.x + 12, p.y}, {p.x + 16, p.y}, col(kTruthCol), 1.5f);
    }

    // Detection, in orange.
    if (r.detected) {
        const ImVec2 p = to_screen(r.detection_img);
        dl->AddLine({p.x - 9, p.y - 9}, {p.x + 9, p.y + 9}, col(kDetectCol), 1.6f);
        dl->AddLine({p.x - 9, p.y + 9}, {p.x + 9, p.y - 9}, col(kDetectCol), 1.6f);
    }

    ImGui::TextColored(kBoreCol, "+ boresight");
    ImGui::SameLine(); ImGui::TextColored(kTruthCol, "  O truth");
    ImGui::SameLine(); ImGui::TextColored(kDetectCol, "  X detection");
    ImGui::SameLine();
    ImGui::TextColored(kMutedCol, "   %d x %d", snap.preview_w, snap.preview_h);

    ImGui::End();
}

void Dashboard::draw_screen_overview() {
    if (dock_.centre_bottom) ImGui::SetNextWindowDockID(dock_.centre_bottom, dock_cond());
    ImGui::Begin("Screen overview");
    ImGui::TextColored(kMutedCol,
        "The %d x %d canvas (specification row 1). The camera sees %.2f%% of it.",
        scenario_.canvas_px[0], scenario_.canvas_px[1],
        100.0 * scenario_.resolution[0] * scenario_.resolution[1]
              / (static_cast<double>(scenario_.canvas_px[0]) * scenario_.canvas_px[1]));

    if (ImPlot::BeginPlot("##overview", ImVec2(-1, -1), ImPlotFlags_Equal)) {
        ImPlot::SetupAxes("screen x (px)", "screen y (px)");
        ImPlot::SetupAxesLimits(0, scenario_.canvas_px[0], scenario_.canvas_px[1], 0,
                                ImPlotCond_Always);

        // The true path.
        if (!truth_path_x_.empty()) {
            ImPlot::SetNextLineStyle(kTruthCol, 1.5f);
            ImPlot::PlotLine("true path", truth_path_x_.data(), truth_path_y_.data(),
                             static_cast<int>(truth_path_x_.size()));
        }
        // What the detector reported. Where this departs from the green line,
        // the detector is wrong — which is the single most legible way to show
        // CP 4.11's failure without reading a number.
        if (!det_path_x_.empty()) {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.5f, kDetectCol, 0.0f, kDetectCol);
            ImPlot::SetNextLineStyle(kDetectCol, 0.0f);
            ImPlot::PlotScatter("reported", det_path_x_.data(), det_path_y_.data(),
                                static_cast<int>(det_path_x_.size()));
        }

        // The camera's field of view, as a rectangle at the true boresight.
        const FrameRecord& r = pipeline_.last();
        const Aabb box = view_aabb(scenario_.camera_geometry(),
                                   scenario_.screen_geometry(), r.boresight_true);
        double bx[5] = {box.x0, box.x1, box.x1, box.x0, box.x0};
        double by[5] = {box.y0, box.y0, box.y1, box.y1, box.y0};
        ImPlot::SetNextLineStyle(kBoreCol, 2.0f);
        ImPlot::PlotLine("field of view", bx, by, 5);

        ImPlot::EndPlot();
    }
    ImGui::End();
}

void Dashboard::draw_error_plots() {
    // -----------------------------------------------------------------------
    // TWO PLOTS, NOT ONE. INV-6: "centroiding error and tracking error are
    // distinct. They are computed separately, logged separately, plotted
    // separately, and reported separately. Never conflate them."
    // -----------------------------------------------------------------------
    if (dock_.right) ImGui::SetNextWindowDockID(dock_.right, dock_cond());
    ImGui::Begin("Centroiding error  (graded, 60%)");
    ImGui::TextColored(kMutedCol,
        "|reported beacon centre - true centre|.  RMSE %.3f px over %zu frames",
        centroid_image_.rmse(), centroid_image_.count());

    if (ImPlot::BeginPlot("##centroid", ImVec2(-1, -1))) {
        // Log on the error axis because the interesting range spans four
        // decades: a working detector sits at ~0.1 px and a failing one at
        // ~1000 px, and a linear axis showing both makes the working case a
        // flat line on the floor.
        //
        // The limits must be strictly POSITIVE. A log axis given a lower bound
        // of zero produces the 1e-252 tick labels and an empty plot, which is
        // what the first version of this did.
        ImPlot::SetupAxes("time (s)", "error (px)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.01, 1000.0, ImPlotCond_Always);

        centroid_image_.flatten(plot_x_, plot_y_);
        if (!plot_x_.empty()) {
            ImPlot::SetNextLineStyle(kDetectCol, 1.6f);
            ImPlot::PlotLine("image frame (the detector)",
                             plot_x_.data(), plot_y_.data(),
                             static_cast<int>(plot_x_.size()));
        }
        centroid_screen_.flatten(plot_x_, plot_y2_);
        if (!plot_x_.empty()) {
            ImPlot::SetNextLineStyle(kMutedCol, 1.2f);
            ImPlot::PlotLine("screen frame (+ pointing error)",
                             plot_x_.data(), plot_y2_.data(),
                             static_cast<int>(plot_x_.size()));
        }
        ImPlot::EndPlot();
    }
    ImGui::SetItemTooltip(
        "The two differ by the pointing error the system cannot measure:\n"
        "jitter (row 23) and platform drift (row 25) move the true boresight,\n"
        "and the encoder does not see them. The image-frame trace is the\n"
        "detector's own accuracy; the screen-frame trace also carries how well\n"
        "we know where we were looking.");
    ImGui::End();

    if (dock_.right_mid) ImGui::SetNextWindowDockID(dock_.right_mid, dock_cond());
    ImGui::Begin("Tracking error  (specification row 17)");
    ImGui::TextColored(kMutedCol,
        "|camera boresight - true beacon angle|.  RMS %.2f px, worst %.1f px",
        tracking_.rmse(), worst_tracking_);

    if (ImPlot::BeginPlot("##tracking", ImVec2(-1, -1))) {
        // AutoFit on time so the trace scrolls with the run; a fixed y range so
        // the 10 px budget line stays in the same place on screen and a viewer
        // can judge compliance at a glance rather than re-reading the axis.
        ImPlot::SetupAxes("time (s)", "error (px)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 40, ImPlotCond_Once);

        tracking_.flatten(plot_x_, plot_y_);
        if (!plot_x_.empty()) {
            ImPlot::SetNextLineStyle(kBoreCol, 1.8f);
            ImPlot::PlotLine("tracking error", plot_x_.data(), plot_y_.data(),
                             static_cast<int>(plot_x_.size()));
        }
        // The 10 px budget, drawn as a line so a viewer can see compliance
        // rather than compute it. Design §12 asks for exactly this.
        double budget = scenario_.tracking_error_px;
        ImPlot::SetNextLineStyle(kBudgetCol, 2.0f);
        ImPlot::PlotInfLines("10 px budget (row 17)", &budget, 1, ImPlotInfLinesFlags_Horizontal);
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void Dashboard::draw_metrics() {
    if (dock_.right_bottom) ImGui::SetNextWindowDockID(dock_.right_bottom, dock_cond());
    ImGui::Begin("Compliance");
    ImGui::TextColored(kMutedCol,
        "Live figures against the specification. Full matrix at Stage 7.");
    ImGui::Spacing();

    const double lock = frames_in_fov_ ? static_cast<double>(frames_detected_)
                                       / static_cast<double>(frames_in_fov_) : 0.0;
    const double loss = 1.0 - std::min(1.0, lock);
    const double fa   = frames_ ? static_cast<double>(false_alarms_)
                                / static_cast<double>(frames_) : 0.0;

    if (ImGui::BeginTable("compliance", 3,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("metric");
        ImGui::TableSetupColumn("value");
        ImGui::TableSetupColumn("requirement");
        ImGui::TableHeadersRow();

        metric_row("Tracking error (RMS)", "%.2f px", tracking_.rmse(),
                   tracking_.rmse() <= scenario_.tracking_error_px, "row 17: <= 10 px");
        metric_row("Tracking error (worst)", "%.1f px", worst_tracking_,
                   worst_tracking_ <= scenario_.tracking_error_px, "row 17");
        metric_row("Centroiding RMSE (image)", "%.3f px", centroid_image_.rmse(),
                   centroid_image_.rmse() < 1.0, "graded, 60% of BP-1/BP-2");
        metric_row("Centroiding RMSE (screen)", "%.3f px", centroid_screen_.rmse(),
                   centroid_screen_.rmse() < 1.0, "graded");
        metric_row("Target loss", "%.1f %%", 100.0 * loss,
                   loss < scenario_.target_loss_frac, "row 18: < 5 %");
        metric_row("False alarms", "%.1f %%", 100.0 * fa,
                   fa < 0.05, "detections with no beacon in view");
        metric_row("Gimbal saturation", "%.1f %%",
                   100.0 * pipeline_.gimbal().saturation_frac(),
                   true, "fraction of ticks at the rate limit");

        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Per-stage timing — percentiles, never means");
    if (ImGui::BeginTable("timing", 4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("stage");
        ImGui::TableSetupColumn("p50 (us)");
        ImGui::TableSetupColumn("p95 (us)");
        ImGui::TableSetupColumn("p99 (us)");
        ImGui::TableHeadersRow();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Stage::kCount); ++i) {
            const auto s = static_cast<Stage>(i);
            const LatencyHistogram& h = pipeline_.timers()[s];
            if (h.count() == 0) continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(stage_name(s));
            ImGui::TableNextColumn(); ImGui::Text("%.1f", h.p50());
            ImGui::TableNextColumn(); ImGui::Text("%.1f", h.p95());
            ImGui::TableNextColumn(); ImGui::Text("%.1f", h.p99());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// draw_tracking_panel — Stage 6 made visible.
//
// Three things, and the third is the one that is hard to get any other way.
//
//   The mode and the track state, side by side. They are different machines
//   (control/mode_fsm.hpp explains why) and seeing them disagree — Reacquire
//   with a Coasting track, say — is how their interaction becomes obvious.
//
//   The filter's own numbers: the velocity estimate that feeds §10.4's
//   feedforward, the position sigma that IS the gate's radius, and the
//   normalised innovation squared, which should sit near 2 on a consistent
//   filter and is the only health check available with no ground truth.
//
//   The transition log, newest first. CP 6.6's acceptance criterion is a
//   trace, and a trace with reasons attached turns "it lost lock somewhere
//   around there" into a line naming the frame and the rule that fired.
// ---------------------------------------------------------------------------
void Dashboard::draw_tracking_panel() {
    if (dock_.left_bottom) ImGui::SetNextWindowDockID(dock_.left_bottom, dock_cond());
    ImGui::Begin("Tracking");

    const Track&  trk  = pipeline_.tracker().track();
    const ModeFsm& fsm = pipeline_.fsm();
    const double  ifov = scenario_.camera_geometry().ifov_urad();

    const bool locked = trk.drivable();
    ImGui::TextUnformatted("mode");
    ImGui::SameLine(140.0f);
    ImGui::TextColored(locked ? ImVec4{0.4f, 0.9f, 0.4f, 1.0f}
                              : ImVec4{1.0f, 0.75f, 0.3f, 1.0f},
                       "%s", track_mode_name(fsm.mode()));
    ImGui::SameLine();
    ImGui::TextColored(kMutedCol, "  track: %s", track_state_name(trk.state()));

    ImGui::TextColored(kMutedCol, "%d frames in mode", fsm.frames_in_mode());
    ImGui::Spacing();

    if (ImGui::BeginTable("filter", 2,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        auto row = [](const char* k, const char* fmt, auto... v) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%s", k);
            ImGui::TableNextColumn(); ImGui::Text(fmt, v...);
        };
        row("candidates",  "%d", pipeline_.last().candidate_count);
        row("detection SNR", "%.1f", static_cast<double>(pipeline_.last().detection_snr));
        row("velocity",    "%.1f, %.1f px/s",
            trk.rate().x / ifov, trk.rate().y / ifov);
        // The gate's radius, in the units a person can check against the camera
        // view. sqrt(chi2) sigmas is what "d^2 < 9.21" means geometrically.
        row("position sigma", "%.2f px", trk.position_sigma_urad() / ifov);
        row("gate radius", "%.2f px",
            std::sqrt(kGateChi2_2dof_99)
                * std::sqrt(trk.position_sigma_urad()
                            * trk.position_sigma_urad()
                          + trk.last_sigma_urad() * trk.last_sigma_urad()) / ifov);
        row("NIS (want ~2)", "%.2f",
            trk.uses_imm() ? trk.imm().last_nis() : trk.filter().last_nis());
        row("hits / age",  "%d / %d", trk.hits(), trk.age_frames());
        row("misses",      "%d", trk.consecutive_misses());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Mode transitions (CP 6.6) — newest first");
    if (ImGui::BeginTable("transitions", 3,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY
                        | ImGuiTableFlags_SizingStretchProp,
                          ImVec2(0, 160))) {
        ImGui::TableSetupColumn("frame");
        ImGui::TableSetupColumn("transition");
        ImGui::TableSetupColumn("why");
        ImGui::TableHeadersRow();
        const auto& log = fsm.log();
        for (size_t i = 0; i < log.size(); ++i) {
            const ModeTransition& t = log.at_back(i);   // newest first
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%lld", static_cast<long long>(t.frame));
            ImGui::TableNextColumn();
            ImGui::Text("%s -> %s", track_mode_name(t.from), track_mode_name(t.to));
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%s", t.reason);
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void Dashboard::draw_scenario_panel() {
    if (dock_.left_bottom) ImGui::SetNextWindowDockID(dock_.left_bottom, dock_cond());
    ImGui::Begin("Scenario");
    ImGui::Text("%s", scenario_.name.c_str());
    ImGui::TextWrapped("%s", scenario_.description.c_str());
    ImGui::Separator();

    // Every row cites its specification number, which is CP 3.6's requirement
    // made visible: you can point at any row of the parameter table and see the
    // value this run is using.
    if (ImGui::BeginTable("params", 3,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("spec");
        ImGui::TableSetupColumn("parameter");
        ImGui::TableSetupColumn("value");
        ImGui::TableHeadersRow();

        auto row = [](const char* spec, const char* name, const char* fmt, ...) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%s", spec);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            va_list args;
            va_start(args, fmt);
            ImGui::TextV(fmt, args);
            va_end(args);
        };

        row("row 1",  "screen",          "%d x %d px", scenario_.canvas_px[0], scenario_.canvas_px[1]);
        row("row 3",  "camera",          "%d x %d px", scenario_.resolution[0], scenario_.resolution[1]);
        row("row 4",  "field of view",   "%.1f x %.1f deg", scenario_.fov_deg[0], scenario_.fov_deg[1]);
        row("row 5",  "camera rate",     "%d Hz", scenario_.camera_hz);
        row("row 10", "target size",     "%d px", scenario_.targets.empty() ? 0 : scenario_.targets[0].size_px);
        row("row 13", "max pan",         "%.1f deg/s", scenario_.max_pan_dps);
        row("row 14", "max tilt",        "%.1f deg/s", scenario_.max_tilt_dps);
        row("row 15", "control rate",    "%d Hz", scenario_.control_hz);
        row("row 21", "salt & pepper",   "%.0f %%", 100.0 * scenario_.salt_pepper);
        row("row 22", "read noise",      "sigma %.0f", scenario_.gaussian_sigma);
        row("row 23", "jitter",          "%.0f px/frame", scenario_.jitter_px_per_frame);
        row("row 24", "atmosphere",      "%s", atmosphere_name(scenario_.atmosphere));
        row("row 25", "platform",        "%zu component(s)", scenario_.platform.size());
        row("§9.1",   "clutter",         "%d sources, %d decoy",
            scenario_.static_sources, scenario_.decoy_beacons);

        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Derived (design §1.4)");
    const auto cam = scenario_.camera_geometry();
    ImGui::Text("IFOV              %.3f urad/px", cam.ifov_urad());
    ImGui::Text("camera authority  %.1f px/frame",
                deg_to_urad(scenario_.max_pan_dps) / cam.ifov_urad() / scenario_.camera_hz);
    ImGui::Text("jitter            %.0f%% of authority",
                100.0 * scenario_.jitter_px_per_frame
                      / (deg_to_urad(scenario_.max_pan_dps) / cam.ifov_urad()
                         / scenario_.camera_hz));
    ImGui::Text("emitters in world %zu", pipeline_.source().emitters().n);
    ImGui::End();
}

// ===========================================================================
// run
// ===========================================================================

int Dashboard::run(const Scenario& initial) {
    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) {
        std::fprintf(stderr, "sat-tracker: could not initialise GLFW.\n"
                             "  On a headless machine use --headless instead.\n");
        return 1;
    }

    // GL 3.3 core is what design §4 names. ImGui's backend needs nothing more,
    // and this file's own calls are all GL 1.1.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    window_ = glfwCreateWindow(1900, 1100, "SAT — Satellite Adaptive Tracker", nullptr, nullptr);
    if (!window_) {
        std::fprintf(stderr, "sat-tracker: could not create a window.\n"
                             "  No display, or no OpenGL 3.3. Use --headless instead.\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);        // vsync: the display has no reason to run faster

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // No imgui.ini: a demo should look the same every time it is opened, not
    // inherit a layout from whatever was last dragged around.
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();

    // HiDPI. On a 2880x1800 laptop panel the default 13 px font is unreadable
    // from more than arm's length, and §14.1's demo is watched from a room
    // rather than a desk. GLFW reports the monitor's content scale; the style
    // is scaled to match and the font rasterised at the scaled size so it stays
    // sharp rather than being a magnified 13 px bitmap.
    float xscale = 1.0f, yscale = 1.0f;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor()) {
        glfwGetMonitorContentScale(mon, &xscale, &yscale);
    }
    const float ui_scale = std::max(1.0f, std::max(xscale, yscale));
    if (ui_scale > 1.0f) {
        ImGui::GetStyle().ScaleAllSizes(ui_scale);
    }
    // A little above 1.0 even on a non-HiDPI screen: the panels carry a lot of
    // small numeric text and legibility matters more here than density.
    ImFontConfig fc;
    fc.SizePixels = 15.0f * ui_scale;
    io.Fonts->AddFontDefault(&fc);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    rebuild(initial);

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        if (running_ || step_once_) {
            const int n = step_once_ ? 1 : steps_per_draw_;
            for (int i = 0; i < n && !finished_; ++i) step_simulation();
            step_once_ = false;
        }
        // Publish/acquire through the triple buffer even single-threaded, so
        // the seam §6.2 B30/B31 describes is genuinely exercised rather than
        // merely present.
        (void)pipeline_.snapshots().acquire();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Order matters: submit the dockspace first, then rearrange it on the
        // first frame. Building the layout before the node exists leaves it to
        // be overwritten when DockSpaceOverViewport creates its own.
        //
        // The layout is also deferred until the viewport has a real size. On
        // the very first frame WorkSize can still be zero, and splitting a
        // zero-sized node collapses every panel to nothing.
        const ImGuiID dockspace =
            ImGui::DockSpaceOverViewport(ImGui::GetID("SatDockSpace"),
                                         ImGui::GetMainViewport(),
                                         ImGuiDockNodeFlags_None);
        // -------------------------------------------------------------------
        // Build the layout ONCE, after the window size has settled.
        //
        // Dock node sizes are absolute pixels fixed at build time, so a layout
        // computed for the size we asked for is wrong if something resizes the
        // window afterwards — and a tiling window manager does exactly that,
        // immediately, turning a requested 1900 px into whatever the tile is.
        // Building at 1900 and then being handed 941 left the centre column a
        // sliver.
        //
        // Rebuilding on every resize looks like the fix and is not: rebuilding
        // destroys the nodes, which undocks every window, and the dock
        // assignments below are ImGuiCond_Once so they do not reapply. The
        // panels end up floating again.
        //
        // Waiting for stability avoids both. A few identical frames in a row
        // means the window manager has finished with us; after that ImGui
        // rescales the nodes proportionally on its own, which is what we want
        // for a user dragging the window edge.
        // -------------------------------------------------------------------
        const ImVec2 work = ImGui::GetMainViewport()->WorkSize;
        if (!layout_built_) {
            const bool same = std::fabs(work.x - layout_size_.x) < 1.0f
                           && std::fabs(work.y - layout_size_.y) < 1.0f;
            stable_frames_ = same ? stable_frames_ + 1 : 0;
            layout_size_.x = work.x;
            layout_size_.y = work.y;
            if (stable_frames_ >= 8 && work.x > 32.0f && work.y > 32.0f) {
                build_default_layout(dockspace, dock_);
                layout_built_ = true;
                // Re-dock for a couple of frames: a rebuild destroys the nodes
                // and undocks every window, and ImGuiCond_Once would not
                // reapply to windows that already exist.
                redock_frames_ = 3;
            }
        }
        draw_menu_bar();
        draw_controls();
        draw_camera_view();
        draw_screen_overview();
        draw_error_plots();
        draw_metrics();
        draw_tracking_panel();
        draw_scenario_panel();

        ImGui::Render();
        int w = 0, h = 0;
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (redock_frames_ > 0) --redock_frames_;
        glfwSwapBuffers(window_);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
    return 0;
}

int run_dashboard(const Scenario& sc) {
    Dashboard d;
    return d.run(sc);
}

}  // namespace sat::gui
