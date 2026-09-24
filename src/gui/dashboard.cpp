// gui/dashboard.cpp — see dashboard.hpp for what this is for.

#include "gui/dashboard.hpp"

#if SAT_HAVE_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

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

// ---------------------------------------------------------------------------
// apply_clean_preset — what "Clean" means, in ONE place.
//
// It used to mean two different things. The "Clean" BUTTON and the state the
// dashboard OPENS in were written separately, and both quietened the damage
// chain only — rows 21, 22 and 24. Rows 23 and 25 are not part of that chain:
// §9.3 puts them on the TRUE BORESIGHT, never on the pixels, so nothing in the
// damage panel reached them and neither path ever turned them off.
//
// The consequence was the most misleading thing on the dashboard. The button's
// tooltip said "the loop should track to a few pixels"; the tracking panel
// then showed about 17 px against a 10 px budget line, on the one preset where
// the loop is supposed to look its best. Row 23's jitter is redrawn every frame
// and added where the controller cannot see it, so it leaves a 16.33 px floor
// under row 17 (design §1.3) that no loop and no predictor can get below. A
// viewer has no way to tell that floor from a system that does not work, and
// the dashboard was actively telling them it was the latter.
//
// Clean now means clean: rows 21 through 25, all quiet. §14.1's demo then works
// as written — open with the loop alone with the target at a few pixels, dial
// each row up, and watch the error respond — and row 23's slider is the single
// most informative control on the panel, because the jump from 4 px to 17 px
// when it goes back to 20 IS the bound, demonstrated rather than asserted.
// ---------------------------------------------------------------------------
void Dashboard::apply_clean_preset() {
    SensorChain& c = pipeline_.source().sensor();
    c.set_atmosphere(Atmosphere::Clear);
    c.noise().gaussian_sigma  = 0.0;
    c.noise().salt_pepper_p   = 0.0;
    c.noise().poisson_enabled = false;
    c.set_defects_enabled(false);

    DisturbanceGenerator& d = pipeline_.source().disturbance();
    d.set_jitter_px_per_frame(0.0);
    d.set_platform_enabled(false);
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
    // A live demo should not die when the scenario clock runs out. The figure-8
    // file is 20 s, which is only a few seconds of wall time on a high-refresh
    // display, and the window then sits on the last frame. Pause is the stop.
    if (continuous_) pipeline_.source().set_continuous(true);
    // P1-2: the dashboard reads the published preview every frame to draw the
    // camera view, so it is the one consumer that needs the copy. Headless
    // runs skip it and save 307 KB per frame. Set AFTER the build, which
    // resets the pipeline's configuration.
    pipeline_.set_preview_consumers(true);

    // Open on the CLEAN preset rather than on the scenario's full damage.
    //
    // THE REASON THIS COMMENT USED TO GIVE IS NO LONGER TRUE, and it is worth
    // recording why rather than silently rewriting it. It said the default
    // detector was "the Stage 1 straw man, and at row 21's 10% impulse noise
    // it fails outright (CP 4.11)". The default detector has been Classical
    // since Stage 4 — the straw man survives only as an ablation arm, selected
    // from the Detector menu — and Classical handles row 21's impulse noise
    // through its median stage at 0.20 px.
    //
    // The DECISION is still right, for a different and better reason: §14.1's
    // running order. A demonstration that opens on full damage shows a number
    // and no context. One that opens clean and dials damage up shows the
    // system responding, which is the only way to tell a working adaptive loop
    // from a lucky constant. The judge watches the error curve move.
    //
    // It is also not flattery, and the GUI now makes sure it cannot be
    // mistaken for it: the damage group carries a persistent "DAMAGE
    // OVERRIDDEN: CLEAN" banner while this preset is active, every row of the
    // specification table reads the LIVE chain rather than the scenario file,
    // and one click on "Full spec" restores the scenario exactly as written.
    if (start_clean_) apply_clean_preset();

    // EVERY trace, not four of them.
    //
    // The four below were cleared and the five above were not, so after a
    // Reset the IMM mode-probability plot, the priority policy's two score
    // traces and the SAT strategy timeline still held the previous run — and
    // the new run's samples were appended to it on a time axis that had just
    // restarted at 0.03 s. The result is a plot showing two runs at once with
    // nothing saying so, which is the same class of mistake as the compliance
    // panel keeping its own counters (P1-8): a dashboard that disagrees with
    // itself is worse than one that shows less.
    centroid_image_.clear();
    centroid_screen_.clear();
    tracking_.clear();
    saturation_.clear();
    imm_cv_.clear(); imm_ca_.clear(); imm_ct_.clear();
    score_committed_.clear(); score_rival_.clear();
    strategy_marks_.clear();
    truth_path_x_.clear(); truth_path_y_.clear();
    det_path_x_.clear();   det_path_y_.clear();

    frames_ = frames_detected_ = frames_in_fov_ = false_alarms_ = 0;
    worst_tracking_ = 0.0;
    finished_ = false;

    // The compliance panel's numbers come from the one MetricCollector, the
    // same one the headless summary and run.json use (P1-8). Reserved for the
    // whole run so add() never allocates — the dashboard is inside the
    // per-frame window that INV-4's trap watches.
    metrics_.begin(scenario_.name, scenario_.seed,
                   scenario_.camera_geometry().ifov_urad(),
                   static_cast<size_t>(scenario_.duration_s * scenario_.camera_hz) + 2,
                   pipeline_.video() == nullptr || pipeline_.video()->supports_pointing());
    metrics_snapshot_ = RunMetrics{};
    metrics_dirty_    = true;

    char buf[256];
    std::snprintf(buf, sizeof(buf), "loaded '%s'  %dx%d px screen, %dx%d camera, %.0f Hz",
                  scenario_.name.c_str(), scenario_.canvas_px[0], scenario_.canvas_px[1],
                  scenario_.resolution[0], scenario_.resolution[1],
                  static_cast<double>(scenario_.camera_hz));
    status_ = buf;
}

const RunMetrics& Dashboard::live_metrics() {
    if (metrics_dirty_) {
        // finish() sorts several sample vectors, so it is recomputed once per
        // SIMULATION frame rather than once per rendered frame — at
        // steps_per_draw_ = 1 those coincide, and at 8 it is eight times less
        // work for the same answer.
        metrics_snapshot_ = metrics_.finish(pipeline_.timers(), /*wall_time_s=*/0.0,
                                            pipeline_.gimbal().saturation_frac());
        metrics_dirty_ = false;
    }
    return metrics_snapshot_;
}

void Dashboard::step_simulation() {
    if (finished_) return;
    if (!pipeline_.step()) {
        finished_ = true;
        running_  = false;
        status_   = "run complete - press Reset to run it again";
        return;
    }

    const FrameRecord& r = pipeline_.last();
    // The collector keeps every sample so it can report an exact p95. That is
    // right for a timed run and wrong for a demo that never ends: finish()
    // sorts the whole series every frame. Stop feeding it at the scenario's
    // own length. The rolling plots above keep updating.
    const int64_t scored = static_cast<int64_t>(scenario_.duration_s * scenario_.camera_hz);
    if (!continuous_ || frames_ < scored) {
        metrics_.add(r);
        metrics_dirty_ = true;
    }

    // Kept alongside the collector, and only for things the collector does not
    // report: `frames_detected_` is a DETECTION rate, which is a useful live
    // readout and is not any of §13.1's ratios, and the rest feed the plots.
    ++frames_;
    if (r.detected)     ++frames_detected_;
    if (r.truth_in_fov) ++frames_in_fov_;
    if (r.false_alarm)  ++false_alarms_;
    worst_tracking_ = std::max(worst_tracking_, r.tracking_error_px);

    tracking_.push(r.time_s, r.tracking_error_px);
    saturation_.push(r.time_s, pipeline_.gimbal().saturation_frac());

    // CP 15.1's panels. Pushed here rather than read on demand because a plot
    // needs the HISTORY, and the pipeline only keeps the current frame.
    const Tracker& tk = pipeline_.tracker();
    if (pipeline_.tracker().track().uses_imm()) {
        imm_cv_.push(r.time_s, static_cast<double>(r.imm_mode_prob[0]));
        imm_ca_.push(r.time_s, static_cast<double>(r.imm_mode_prob[1]));
        imm_ct_.push(r.time_s, static_cast<double>(r.imm_mode_prob[2]));
    }
    score_committed_.push(r.time_s, static_cast<double>(tk.committed_score()));
    score_rival_.push(r.time_s, static_cast<double>(tk.best_rival_score()));

    if (r.sup_switched) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "%s  k=%.2f  gate=%.2f",
                      centroid_kind_name(r.sup_centroider),
                      static_cast<double>(r.sup_cfar_k),
                      static_cast<double>(r.sup_cfar_k) * 1.5);
        strategy_marks_.push_back(StrategyMark{r.time_s, buf,
                                               r.sup_snr, r.sup_q_scale});
        // §10.6's dwell allows at most one switch per second, so 2048 is over
        // half an hour of demo. Dropping the oldest keeps the panel bounded
        // without the timeline lying about what happened recently.
        if (strategy_marks_.size() > 2048) strategy_marks_.erase(strategy_marks_.begin());
    }
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
        // -------------------------------------------------------------------
        // The shipped scenarios, loadable live, IN TWO GROUPS.
        //
        // The split is not cosmetic. This menu used to open with
        // baseline.toml — a scenario that scores 913 px tracking RMS, zero
        // centroiding frames and 1778 false tracks a minute — listed first and
        // unlabelled, directly beneath a comment calling it the known-good
        // fallback. A judge picking the top entry saw the worst run in the
        // repository and had no way to know that was the intent.
        //
        // Now the working scenarios are first and the failing ones are behind
        // a separator that says what they are. Both groups are still one click
        // away: hiding the hard cases would be the opposite mistake, and
        // docs/DEMO.md step 6 deliberately runs one in front of the judges.
        // -------------------------------------------------------------------
        auto load = [&](const char* name) {
            if (!ImGui::MenuItem(name)) return;
            auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/" + name);
            if (r) rebuild(*r);
            else   status_ = "load failed: " + r.error();
        };

        // CP 15.3's "known-good scenario preloaded as a fallback". This one
        // really is known-good: 0.20 px centroiding, 0.067 s acquisition,
        // 100 % FOV containment, rows 16/18/19/20 all PASS.
        load("spec_defaults.toml");
        load("fog_figure8.toml");
        load("maxnoise_random.toml");

        ImGui::Separator();
        ImGui::TextColored(kMutedCol, "Hard cases - known failures");
        // Each of these fails, each fails for ONE reason, and each says so in
        // its own file header with the measured numbers.
        load("hard/clutter_field.toml");            // discrimination
        load("hard/cold_start_in_clutter.toml");    // + acquisition geometry

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
        "so a random beacon starts off-screen 92%% of the time - and finding it\n"
        "is the search problem, which is Stage 13 and not built yet.\n\n"
        "Turn this off to see exactly why Stage 13 is needed.");
    if (continuous_) {
        ImGui::TextColored(kMutedCol,
            "frame %lld   t = %.2f s   runs until Pause",
            static_cast<long long>(frames_),
            pipeline_.last().time_s);
    } else {
        ImGui::TextColored(kMutedCol,
            "frame %lld of %lld   t = %.2f s",
            static_cast<long long>(frames_),
            static_cast<long long>(scenario_.duration_s * scenario_.camera_hz),
            pipeline_.last().time_s);
    }

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

    // -----------------------------------------------------------------------
    // CP 15.2: live algorithm switching.
    //
    // The checkpoint's criterion is literal — "you can turn feedforward off
    // live and watch the error trace blow up" — and §14.1 gives it 90 seconds
    // of the demo. It is the single most convincing thing in the presentation
    // because the effect is instant and enormous: CP 10.1 measures 21.60 px
    // against 2.53 px on a 200 px/s target, and the trace separates within a
    // second of the click.
    //
    // These write through to the LIVE objects rather than rebuilding, which is
    // the whole point: a rebuild would restart the run and the audience would
    // see two runs rather than one loop changing its mind. Gains go through
    // set_gains so the switch is bumpless (§10.6's property 2) — reset() would
    // zero the integrator and kick the mount, which looks like the feedforward
    // mattering when it is the reset.
    // -----------------------------------------------------------------------
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Algorithms - live (CP 15.2)");

    {
        ControlGains g = pipeline_.controller().az().gains();
        bool ff_on = g.k_ff > 0.0;
        ImGui::PushStyleColor(ImGuiCol_Text, ff_on ? ImVec4{0.4f, 0.9f, 0.4f, 1.0f}
                                                   : ImVec4{1.0f, 0.45f, 0.35f, 1.0f});
        if (ImGui::Checkbox("velocity feedforward (CP 10.1)", &ff_on)) {
            g.k_ff = ff_on ? 1.0 : 0.0;
            pipeline_.controller_mut().set_gains(g);
        }
        ImGui::PopStyleColor();
        ImGui::SetItemTooltip(
            "Pure feedback lags by roughly speed / bandwidth. On a 200 px/s\n"
            "target that is v/kp = 25 px before any noise. Measured 21.60 px\n"
            "off, 2.53 px on. Watch the tracking trace, not the camera view -\n"
            "at row 12's 24.6 px/s the lag is 3 px and you will not see it.");

        bool aw = g.anti_windup;
        if (ImGui::Checkbox("anti-windup (CP 10.2)", &aw)) {
            g.anti_windup = aw;
            pipeline_.controller_mut().set_gains(g);
        }
        ImGui::SetItemTooltip(
            "Conditional integration. Visible on an acquisition SLEW, not in\n"
            "steady state: 7.50 px of overshoot with it, 12.06 px without,\n"
            "against specification row 17's 10 px budget.");

        bool smith = g.smith;
        if (ImGui::Checkbox("Smith predictor (CP 10.4)", &smith)) {
            g.smith = smith;
            pipeline_.controller_mut().set_gains(g);
        }
        ImGui::SetItemTooltip(
            "Built, measured, and OFF by default because it does not help on\n"
            "this plant: 20 degrees of delay phase at crossover out of a margin\n"
            "near 90. The loop is not delay-limited. Turning it on makes p95\n"
            "worse, which is worth showing.");
    }

    {
        TrackParams& tp = pipeline_.tracker_mut().params();
        bool imm = tp.imm;
        if (ImGui::Checkbox("IMM: CV/CA/CT (CP 10.5)", &imm)) {
            // NEW tracks only. Swapping a live four-state filter for a
            // six-state one mid-track would have to invent two states, and
            // INV-9's principle applies to a state estimate as much as to a
            // centroid.
            tp.imm = imm;
        }
        ImGui::SetItemTooltip(
            "Takes effect on the next track, not this one - swapping a live\n"
            "4-state filter for a 6-state one would have to invent two states.\n"
            "Worth 21.75 -> 17.15 px on the figure-8, and nothing on a straight\n"
            "line. Load scenarios/control/figure8.toml to see it.");
    }

    // --- damage, dialled live (§14.1, 3:00-5:00) ---------------------------
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Damage - specification rows 21-25");

    SensorChain& chain = pipeline_.source().sensor();

    // ----------------------------------------------------------------------
    // THE OVERRIDE BANNER — P1-8.
    //
    // The dashboard opens on the Clean preset, which is the right call for a
    // demonstration (§14.1's running order: start clean, dial damage up, watch
    // the error respond — that is how you tell a working adaptive loop from a
    // lucky constant). What was wrong was that nothing said so.
    //
    // A judge who walks up to a running dashboard has no way to know whether
    // the damage controls are at the scenario's values or at someone's
    // experiment from two minutes ago. Every number on screen depends on the
    // answer. So: a persistent banner whenever the live chain differs from the
    // file, naming the difference, with the restore button beside it.
    //
    // Persistent rather than a toast, because the ambiguity is persistent.
    // ----------------------------------------------------------------------
    {
        const NoiseParams& n0 = chain.noise();
        const DisturbanceGenerator& d0 = pipeline_.source().disturbance();
        const bool overridden =
               !chain.enabled()
            || chain.atmosphere()    != scenario_.atmosphere
            || n0.poisson_enabled    != scenario_.noise_poisson
            || !chain.defects_enabled()
            || std::abs(n0.gaussian_sigma - scenario_.gaussian_sigma) > 1e-9
            || std::abs(n0.salt_pepper_p  - scenario_.salt_pepper)    > 1e-9
            // Rows 23 and 25. The banner's whole job is "the live chain is not
            // the file", and it could not see the two rows that dominate the
            // graded tracking number.
            || std::abs(d0.jitter_px_per_frame() - scenario_.jitter_px_per_frame) > 1e-9
            || (d0.has_platform() && !d0.platform_enabled());

        if (overridden) {
            const bool clean = !chain.enabled()
                            || (n0.gaussian_sigma == 0.0 && n0.salt_pepper_p == 0.0
                                && !n0.poisson_enabled);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.72f, 0.18f, 1.0f));
            ImGui::TextWrapped(
                clean ? "DAMAGE OVERRIDDEN: CLEAN - this run is NOT at the "
                        "scenario's specification values. Click \"Full spec\" to "
                        "restore them."
                      : "DAMAGE OVERRIDDEN - the live chain differs from the "
                        "scenario file. The Scenario panel shows both. Click "
                        "\"Full spec\" to restore.");
            ImGui::PopStyleColor();
        } else {
            ImGui::TextColored(ImVec4(0.30f, 0.72f, 0.50f, 1.0f),
                               "At the scenario's specification values.");
        }
    }

    // Presets, because §14.1's demo dials damage up in stages and hunting for
    // four sliders mid-presentation is not something anyone should have to do.
    DisturbanceGenerator& dist = pipeline_.source().disturbance();
    if (ImGui::Button("Clean")) apply_clean_preset();
    ImGui::SetItemTooltip(
        "No damage and no disturbance: rows 21-25 all quiet. This is the one\n"
        "preset where the loop is alone with the target, and it tracks to a\n"
        "few pixels. Turn row 23's jitter back up below and watch it jump to\n"
        "~17 px - that is the 16.33 px floor jitter puts under row 17, not a\n"
        "loop that stopped working.");
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
        dist.set_jitter_px_per_frame(scenario_.jitter_px_per_frame);
        dist.set_platform_enabled(true);
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

    // -----------------------------------------------------------------------
    // Rows 23 and 25 — THE DISTURBANCES, and why they belong in this panel.
    //
    // This panel is headed "Damage - specification rows 21-25" and had
    // controls for 21, 22 and 24 only. The scenario panel's table listed rows
    // 23 and 25 as live values against the file's, with the comment "every one
    // of these is live-adjustable from the controls panel" — which was not
    // true of either, so the comparison could never differ.
    //
    // They are the two that decide the graded number. Row 23's jitter is drawn
    // fresh every frame and added to the TRUE boresight, where the controller
    // cannot see it and cannot reject it, so it puts a hard 16.33 px floor
    // under row 17's 10 px budget (design §1.3). Without a control, a viewer
    // looking at 17 px of tracking error has no way to distinguish that floor
    // from a loop that does not work — and the single most useful thing the
    // dashboard can do is let them drag the slider to zero and watch the error
    // fall to ~4 px, which says "the loop is fine and the budget is the
    // specification's problem" far better than a paragraph does.
    // -----------------------------------------------------------------------
    ImGui::Spacing();
    ImGui::TextColored(kMutedCol, "Disturbance - rows 23 and 25 (moves the TRUE boresight)");

    float jit = static_cast<float>(dist.jitter_px_per_frame());
    if (ImGui::SliderFloat("jitter px/frame (row 23)", &jit, 0.0f, 20.0f, "%.1f")) {
        dist.set_jitter_px_per_frame(jit);
    }
    ImGui::SetItemTooltip(
        "Uniform in [-A, +A], redrawn once per camera frame, applied to the\n"
        "boresight the frame is RENDERED at while the tracker is told the\n"
        "commanded one. It is white, so no controller and no predictor can\n"
        "reject it: over two axes it leaves sqrt(2*A*A/3) of pointing error,\n"
        "which at the specification's A = 20 is 16.33 px against row 17's\n"
        "10 px budget. Drag it to 0 and the tracking trace drops to a few px.");

    if (dist.has_platform()) {
        bool plat = dist.platform_enabled();
        if (ImGui::Checkbox("platform motion (row 25)", &plat)) {
            dist.set_platform_enabled(plat);
        }
        ImGui::SetItemTooltip(
            "The mount's carrier drifting under the camera. Unlike jitter it is\n"
            "smooth, so the filter CAN follow it: the measurement is formed\n"
            "through the commanded boresight, which carries the drift, and the\n"
            "loop cancels it without ever being told it exists. Turning it off\n"
            "should barely move the tracking trace - that is the point.");
    } else {
        ImGui::TextColored(kMutedCol, "platform motion (row 25)  -  none in this scenario");
    }

    // Clutter needs a world rebuild, so it sits apart from the live sliders.
    ImGui::Spacing();
    ImGui::TextColored(kMutedCol, "Clutter (design S9.1) - rebuilds the world");
    bool rebuild_needed = false;
    rebuild_needed |= ImGui::SliderInt("static sources", &clutter_sources_, 0, 400);
    rebuild_needed |= ImGui::SliderInt("decoy beacons",  &decoy_beacons_,   0, 4);
    if (rebuild_needed && !ImGui::IsItemActive()) rebuild(scenario_);
    ImGui::SetItemTooltip(
        "S9.1 calls 50-500 static sources \"mandatory for credibility\", and\n"
        "deliberately makes some BRIGHTER than the beacon. Drag this up and\n"
        "watch the brightest-pixel detector lock onto one of them instead -\n"
        "that is CP 4.11.");

    // -----------------------------------------------------------------------
    // The detector, live. CP 15.2 asks for switching you can watch, and this is
    // the switch with the largest visible consequence in the whole project.
    //
    // This block used to read "Detector: brightest pixel - a deliberate straw
    // man ... Stage 5 replaces this with the real pipeline". Stage 5 arrived
    // and the text did not: the dashboard has been running the classical
    // pipeline for some time while telling everyone who looked at it that it
    // was not. Stale explanatory text is worse than none, because it is
    // believed.
    // -----------------------------------------------------------------------
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Detector (CP 4.11's ablation, live)");
    {
        const bool classical =
            pipeline_.config().detector == DetectorKind::Classical;
        if (ImGui::RadioButton("classical (S9.4)", classical)) {
            pipeline_.set_detector(DetectorKind::Classical);
        }
        ImGui::SetItemTooltip(
            "A 3x3 median, a van Herk top-hat, integer summed-area tables, a\n"
            "multi-scale matched filter, CFAR with a guard band, run-length\n"
            "grouping, a shape gate and an SNR gate. This is what ships.");
        ImGui::SameLine();
        if (ImGui::RadioButton("brightest pixel", !classical)) {
            pipeline_.set_detector(DetectorKind::BrightestPixel);
        }
        ImGui::SetItemTooltip(
            "The straw man, kept runnable because CP 4.11's finding is a claim\n"
            "about the CLOSED LOOP and needs both arms. Switch to it with the\n"
            "clutter slider up, or with salt-and-pepper at row 21's 10%%, and\n"
            "watch it lock onto the wrong thing within a few frames.");
    }

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

    // Every row below comes from the SAME MetricCollector the headless summary
    // and run.json use. See the note on Dashboard::metrics_ for what the two
    // parallel implementations used to disagree about (P1-8).
    const RunMetrics& m = live_metrics();
    const double fa = frames_ ? static_cast<double>(false_alarms_)
                              / static_cast<double>(frames_) : 0.0;

    if (ImGui::BeginTable("compliance", 3,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        // Explicit weights. The default is an equal share per column, which
        // left the value column too narrow for "17.31 px" and silently
        // truncated it to "17.31 p" — a compliance panel that clips the number
        // it exists to show. The label and requirement columns are prose and
        // wrap; the value column must not.
        ImGui::TableSetupColumn("metric",      ImGuiTableColumnFlags_WidthStretch, 2.4f);
        ImGui::TableSetupColumn("value",       ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("requirement", ImGuiTableColumnFlags_WidthStretch, 3.0f);
        ImGui::TableHeadersRow();

        // Row 17 on the STEADY-STATE figure, with the transient beside it
        // (P1-9). The panel used to grade the whole-run RMS, which sixty
        // frames into a run is almost entirely the acquisition slew: it read
        // 67.39 px in red while the same run's settled error was 16.9 px.
        metric_row("Tracking error (steady RMS)", "%.2f px", m.tracking_rms_steady_px,
                   m.tracking_rms_steady_px <= scenario_.tracking_error_px,
                   "row 17: <= 10 px");
        metric_row("  - acquisition transient", "%.1f px", m.tracking_rms_transient_px,
                   true, "first 30 frames after lock - not graded");
        metric_row("Tracking error (worst)", "%.1f px", m.tracking_max_px,
                   m.tracking_max_px <= scenario_.tracking_error_px, "row 17");
        metric_row("Centroiding RMSE (image)", "%.3f px", m.centroid_rmse_image_px,
                   m.centroid_rmse_image_px < 1.0, "graded, 60% of BP-1/BP-2");
        // The boresight column rather than the screen one, because the screen
        // figure is dominated by the accumulated pointing error and grows
        // without bound — see METRICS.md §2.3. Both are in centroid.csv.
        metric_row("Centroiding RMSE (boresight)", "%.3f px", m.centroid_rmse_boresight_px,
                   m.centroid_rmse_boresight_px < 1.0, "graded, screen px, drift removed");
        metric_row("  - screen frame", "%.1f px", m.centroid_rmse_screen_px,
                   true, "+ pointing error, unbounded - not a detector metric");

        // The PS's own objective, above the loss rows it conditions.
        metric_row("FOV containment", "%.1f %%", 100.0 * m.fov_containment_frac,
                   m.fov_containment_frac > 0.95,
                   "the PS objective: keep it in the FOV");
        // Row 18 on the post-acquisition denominator (P0-2).
        metric_row("Target loss (post-acq)", "%.1f %%", 100.0 * m.target_loss_post_acq,
                   m.post_acq_valid && m.target_loss_post_acq < scenario_.target_loss_frac,
                   "row 18: < 5 %");
        metric_row("  - over in-FOV frames", "%.1f %%", 100.0 * m.target_loss_frac,
                   true, "detector-centric denominator - not graded");

        metric_row("False alarms", "%.1f %%", 100.0 * fa,
                   fa < 0.05, "detections with no beacon in view");
        metric_row("Gimbal saturation", "%.1f %%",
                   100.0 * pipeline_.gimbal().saturation_frac(),
                   true, "fraction of ticks at the rate limit");

        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Per-stage timing - percentiles, never means");
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
    focus_if_requested("tracking");
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
    ImGui::TextColored(kMutedCol, "Mode transitions (CP 6.6) - newest first");
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

// ===========================================================================
// CP 15.1 — the §12 panels that had nothing to attach to until Stages 6, 10
// and 12 existed.
//
// §14.0a listed four of them as outstanding: "the IMM mode-probability panel,
// the SAT strategy timeline, the mode-FSM graph and live algorithm switching".
// Live switching landed at Stage 15; these are the other three, plus one the
// design did not anticipate because §10.2's policy did not exist in its current
// form — the hypothesis table, which shows the single most consequential
// decision the tracker makes.
// ===========================================================================

// Bring a docked panel to the front of its tab bar, when a screenshot run asked
// for it by name. Docked panels share a tab bar, so a figure OF one has to say
// which one; in an interactive session this does nothing and the user clicks.
void Dashboard::focus_if_requested(const char* title) {
    if (job_.focus.empty()) return;
    if (job_.focus != title) return;
    ImGui::SetNextWindowFocus();
}

void Dashboard::draw_imm_panel() {
    if (dock_.right_bottom) ImGui::SetNextWindowDockID(dock_.right_bottom, dock_cond());
    focus_if_requested("imm");
    ImGui::Begin("IMM mode probabilities  (CP 10.5)");

    const Track& trk = pipeline_.tracker().track();
    if (!trk.uses_imm()) {
        ImGui::TextColored(kMutedCol,
            "The IMM is off. It is opt-in (tracking.imm) because every\n"
            "reproducibility fingerprint in the project is a hash of the\n"
            "simulation state, and a six-state filter would churn all of them.\n\n"
            "Turn it on in Run control, or load scenarios/fog_figure8.toml,\n"
            "where the manoeuvre is what it is for.");
        ImGui::End();
        return;
    }

    // What the checkpoint asks to be visible: the shift at a crossing.
    ImGui::TextColored(kMutedCol,
        "CV constant velocity  -  CA constant acceleration  -  CT coordinated turn");

    if (ImPlot::BeginPlot("##imm", ImVec2(-1, 190))) {
        ImPlot::SetupAxes("t (s)", "probability",
                          ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_Lock);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.0, ImPlotCond_Always);
        auto line = [&](const char* name, const Trace& t, ImVec4 col) {
            if (t.empty()) return;
            t.flatten(plot_x_, plot_y_);
            ImPlot::SetNextLineStyle(col, 2.0f);
            ImPlot::PlotLine(name, plot_x_.data(), plot_y_.data(),
                             static_cast<int>(plot_x_.size()));
        };
        line("CV", imm_cv_, ImVec4{0.45f, 0.75f, 1.00f, 1.0f});
        line("CA", imm_ca_, ImVec4{1.00f, 0.78f, 0.35f, 1.0f});
        line("CT", imm_ct_, ImVec4{0.95f, 0.45f, 0.75f, 1.0f});
        ImPlot::EndPlot();
    }

    const ImmFilter& imm = trk.imm();
    if (ImGui::BeginTable("immnow", 2,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        auto row = [](const char* k, const char* fmt, auto... v) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%s", k);
            ImGui::TableNextColumn(); ImGui::Text(fmt, v...);
        };
        row("P(CV)", "%.3f", imm.mode_prob(ImmMode::CV));
        row("P(CA)", "%.3f", imm.mode_prob(ImmMode::CA));
        row("P(CT)", "%.3f", imm.mode_prob(ImmMode::CT));
        // The turn rate is the CT model's own state, and seeing it move is what
        // makes "the mode plot shows the shift" mean something physical.
        row("turn rate", "%.2f deg/s", imm.turn_rate_rad_s() * 180.0 / kPi);
        ImGui::EndTable();
    }
    ImGui::End();
}

void Dashboard::draw_hypotheses_panel() {
    if (dock_.right_bottom) ImGui::SetNextWindowDockID(dock_.right_bottom, dock_cond());
    focus_if_requested("priority");
    ImGui::Begin("Priority policy  (design 10.2)");

    const Tracker& tk = pipeline_.tracker();
    const double ifov = scenario_.camera_geometry().ifov_urad();

    ImGui::TextColored(kMutedCol,
        "The mount follows ONE track. This is how it is chosen: candidates\n"
        "become hypotheses, and a hypothesis takes the mount only if it MOVES\n"
        "differently from the static world. Clutter does not.");
    ImGui::Spacing();

    if (!tk.weights().enabled) {
        ImGui::TextColored(ImVec4{1.0f, 0.75f, 0.3f, 1.0f},
            "Policy OFF - the strongest candidate in the first frame that has\n"
            "one becomes the track. This is the ablation arm; on the\n"
            "specification's own scenario it locks onto a rock.");
        ImGui::End();
        return;
    }

    // The ego-motion estimate. It is a real physical quantity — spec row 25's
    // platform rate as the tracker sees it — so showing it lets a viewer check
    // it against the scenario.
    const Angle2 cv = tk.common_velocity();
    ImGui::Text("static world drifts at %.1f, %.1f px/s%s",
                cv.x / ifov, cv.y / ifov,
                tk.common_velocity_valid() ? "" : "   (not yet measurable)");
    ImGui::TextColored(kMutedCol,
        "committed score %.3f   best rival %.3f   switch pressure %d/%d   "
        "switches %lld   drops %lld",
        static_cast<double>(tk.committed_score()),
        static_cast<double>(tk.best_rival_score()),
        tk.switch_pressure(), tk.weights().switch_frames,
        static_cast<long long>(tk.switches()), static_cast<long long>(tk.drops()));

    ImGui::Spacing();
    if (ImGui::BeginTable("hyps", 6,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders
                        | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("slot");
        ImGui::TableSetupColumn("state");
        ImGui::TableSetupColumn("SNR");
        ImGui::TableSetupColumn("hits");
        ImGui::TableSetupColumn("rel. speed");
        ImGui::TableSetupColumn("score");
        ImGui::TableHeadersRow();

        const Track& trk = tk.track();
        if (trk.alive()) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextColored(ImVec4{0.4f, 0.9f, 0.4f, 1.0f}, "MOUNT");
            ImGui::TableNextColumn(); ImGui::Text("%s", track_state_name(trk.state()));
            ImGui::TableNextColumn(); ImGui::Text("%.1f", static_cast<double>(trk.mean_snr()));
            ImGui::TableNextColumn(); ImGui::Text("%d/%d", trk.hits(), trk.age_frames());
            ImGui::TableNextColumn();
            ImGui::Text("%.1f px/s", trk.relative_speed_urad_s() / ifov);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", static_cast<double>(tk.committed_score()));
        }
        for (int i = 0; i < Tracker::kMaxHypotheses; ++i) {
            const Track& h = tk.hypothesis(i);
            if (!h.alive()) continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%d", i);
            ImGui::TableNextColumn(); ImGui::Text("%s", track_state_name(h.state()));
            ImGui::TableNextColumn(); ImGui::Text("%.1f", static_cast<double>(h.mean_snr()));
            ImGui::TableNextColumn(); ImGui::Text("%d/%d", h.hits(), h.age_frames());
            ImGui::TableNextColumn();
            ImGui::Text("%.1f px/s", h.relative_speed_urad_s() / ifov);
            ImGui::TableNextColumn();
            const float sc = tk.hypothesis_score(i);
            ImGui::TextColored(sc >= tk.weights().min_commit_score
                                   ? ImVec4{0.4f, 0.9f, 0.4f, 1.0f} : kMutedCol,
                               "%.3f", static_cast<double>(sc));
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::TextColored(kMutedCol,
        "A score needs %.2f to take the mount. The four terms that are not\n"
        "motion sum to %.2f, so brightness and stability alone cannot reach it.",
        static_cast<double>(tk.weights().min_commit_score),
        static_cast<double>(tk.weights().snr + tk.weights().stability
                          + tk.weights().centrality + tk.weights().age));

    if (!score_committed_.empty() && ImPlot::BeginPlot("##scores", ImVec2(-1, 150))) {
        ImPlot::SetupAxes("t (s)", "priority score",
                          ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_Lock);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.05, ImPlotCond_Always);
        score_committed_.flatten(plot_x_, plot_y_);
        ImPlot::SetNextLineStyle(ImVec4{0.4f, 0.9f, 0.4f, 1.0f}, 2.0f);
        ImPlot::PlotLine("committed", plot_x_.data(), plot_y_.data(),
                         static_cast<int>(plot_x_.size()));
        score_rival_.flatten(plot_x_, plot_y2_);
        ImPlot::SetNextLineStyle(ImVec4{1.0f, 0.55f, 0.35f, 1.0f}, 1.5f);
        ImPlot::PlotLine("best rival", plot_x_.data(), plot_y2_.data(),
                         static_cast<int>(plot_y2_.size()));
        const double thr = static_cast<double>(tk.weights().min_commit_score);
        ImPlot::SetNextLineStyle(ImVec4{0.9f, 0.3f, 0.3f, 0.8f}, 1.0f);
        ImPlot::PlotInfLines("commit threshold", &thr, 1, ImPlotInfLinesFlags_Horizontal);
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void Dashboard::draw_strategy_panel() {
    if (dock_.right_bottom) ImGui::SetNextWindowDockID(dock_.right_bottom, dock_cond());
    focus_if_requested("strategy");
    ImGui::Begin("SAT strategy timeline  (CP 12.2)");

    const FrameRecord& r = pipeline_.last();
    if (!pipeline_.config().supervisor.enabled) {
        ImGui::TextColored(kMutedCol,
            "The supervisor is off. It CHANGES the configuration a run uses, so\n"
            "a run with it on and one with it off are different claims and must\n"
            "be distinguishable - the same argument INV-7 makes for --no-ai.\n\n"
            "Turn it on in Run control to watch it adapt.");
        ImGui::End();
        return;
    }

    ImGui::Text("now:  %s   k = %.2f   q x %.2f",
                centroid_kind_name(r.sup_centroider),
                static_cast<double>(r.sup_cfar_k),
                static_cast<double>(r.sup_q_scale));
    ImGui::TextColored(kMutedCol, "deciding on integrated SNR %.1f", 
                       static_cast<double>(r.sup_snr));
    ImGui::Spacing();

    if (strategy_marks_.empty()) {
        ImGui::TextColored(kMutedCol,
            "No switch yet. S10.6's hysteresis allows at most one per second\n"
            "and the middle SNR band deliberately does nothing - a supervisor\n"
            "that always does SOMETHING is a supervisor that is guessing.");
    } else if (ImGui::BeginTable("strat", 3,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY
                               | ImGuiTableFlags_SizingStretchProp, ImVec2(0, 200))) {
        ImGui::TableSetupColumn("t (s)");
        ImGui::TableSetupColumn("chose");
        ImGui::TableSetupColumn("because SNR was");
        ImGui::TableHeadersRow();
        for (size_t i = strategy_marks_.size(); i-- > 0;) {   // newest first
            const StrategyMark& m = strategy_marks_[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%.2f", m.t_s);
            ImGui::TableNextColumn(); ImGui::Text("%s", m.label.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(kMutedCol, "%.1f", static_cast<double>(m.snr));
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void Dashboard::draw_mode_graph() {
    if (dock_.left_bottom) ImGui::SetNextWindowDockID(dock_.left_bottom, dock_cond());
    focus_if_requested("fsm");
    ImGui::Begin("Mode FSM  (design 10.4)");

    // §14.0's substitute for this panel was "assert the FSM transition sequence
    // against the expected trace", and that test stays — it tests more than a
    // person watching. This is the inspection surface: it makes the CURRENT
    // state and the shape of the machine legible at a glance, which a trace in
    // a log does not.
    const TrackMode now = pipeline_.fsm().mode();

    struct Node { TrackMode mode; float x, y; const char* label; };
    static constexpr Node kNodes[] = {
        {TrackMode::Search,    0.10f, 0.50f, "Search"},
        {TrackMode::Detect,    0.32f, 0.50f, "Detect"},
        {TrackMode::Acquire,   0.54f, 0.50f, "Acquire"},
        {TrackMode::Track,     0.76f, 0.50f, "Track"},
        {TrackMode::Reacquire, 0.76f, 0.15f, "Reacquire"},
        {TrackMode::Handover,  0.95f, 0.50f, "Handover"},
        {TrackMode::Safe,      0.32f, 0.85f, "Safe"},
        {TrackMode::Idle,      0.10f, 0.15f, "Idle"},
    };
    struct Edge { int from, to; };
    static constexpr Edge kEdges[] = {
        // Design §10.4's arrows, plus the two §14.0c added because the table
        // stops at the happy path: Detect and Acquire both fall back to Search.
        {7,0}, {0,1}, {1,2}, {2,3}, {3,5}, {3,4}, {4,3}, {4,0}, {1,0}, {2,0},
        {3,6},
    };

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  w = avail.x;
    const float  h = std::max(150.0f, std::min(avail.y, 220.0f));
    ImDrawList*  dl = ImGui::GetWindowDrawList();

    auto at = [&](const Node& n) {
        return ImVec2(origin.x + n.x * w, origin.y + n.y * h);
    };

    for (const Edge& e : kEdges) {
        const ImVec2 a = at(kNodes[e.from]);
        const ImVec2 b = at(kNodes[e.to]);
        dl->AddLine(a, b, IM_COL32(120, 130, 150, 160), 1.5f);
    }
    for (const Node& n : kNodes) {
        const bool live = (n.mode == now);
        const ImVec2 c  = at(n);
        const float  r  = 26.0f;
        dl->AddCircleFilled(c, r, live ? IM_COL32(60, 170, 90, 230)
                                       : IM_COL32(45, 50, 60, 220));
        dl->AddCircle(c, r, live ? IM_COL32(120, 240, 150, 255)
                                 : IM_COL32(110, 120, 140, 200), 0, live ? 2.5f : 1.0f);
        const ImVec2 ts = ImGui::CalcTextSize(n.label);
        dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f),
                    live ? IM_COL32(255, 255, 255, 255) : IM_COL32(190, 200, 215, 255),
                    n.label);
    }
    ImGui::Dummy(ImVec2(w, h));

    ImGui::TextColored(kMutedCol, "%d frames in %s",
                       pipeline_.fsm().frames_in_mode(), track_mode_name(now));
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

        // ------------------------------------------------------------------
        // EVERY ROW READS THE LIVE CHAIN — P1-8.
        //
        // Rows 21 to 25 used to read the SCENARIO FILE while the clutter row
        // beside them read the live world, and the dashboard opens on the
        // Clean preset (see rebuild()). So the same window could print
        //
        //     row 21  salt & pepper   10 %
        //     row 22  read noise      sigma 20
        //     §9.1    clutter         0 sources, 0 decoy
        //
        // beside a compliance panel reading "Centroiding RMSE 0.084 px" — on a
        // run with no noise at all. A judge reading that window concludes the
        // system achieves 0.084 px under 10 % impulse noise and sigma-20 read
        // noise. It does not: measured under those conditions it is 0.497 px,
        // and at p95 across haze/fog/rain about 5.4 px.
        //
        // Nothing in the window was lying on its own. The table was reporting
        // what the file asked for and the compliance panel what the run
        // achieved, and put together they made a claim neither of them made.
        //
        // Now every row reports what is ACTUALLY RUNNING, and a row that
        // differs from the file says so with the file's value in parentheses.
        // A banner above the damage controls names the override, so the state
        // is never ambiguous and one click restores it.
        // ------------------------------------------------------------------
        const SensorChain&    chain = pipeline_.source().sensor();
        const NoiseParams&    ln    = chain.noise();
        const EmitterSoA&     em    = pipeline_.source().emitters();

        int live_clutter = 0, live_decoy = 0;
        for (size_t i = 0; i < em.n; ++i) {
            const EmitterKind k = em.kind_of(i);
            if      (k == EmitterKind::Clutter) ++live_clutter;
            else if (k == EmitterKind::Decoy)   ++live_decoy;
        }

        // A row whose live value differs from the file's. The file's value is
        // kept in view rather than replaced: "what did I ask for" and "what am
        // I getting" are both questions someone asks at a demo.
        auto row_live = [&](const char* spec, const char* name,
                            const std::string& live, const std::string& file) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(kMutedCol, "%s", spec);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(name);
            ImGui::TableNextColumn();
            if (live == file) {
                ImGui::TextUnformatted(live.c_str());
            } else {
                ImGui::TextColored(ImVec4(0.85f, 0.65f, 0.15f, 1.0f), "%s", live.c_str());
                ImGui::SameLine();
                ImGui::TextColored(kMutedCol, "(file: %s)", file.c_str());
            }
        };
        auto fmt = [](const char* f, auto... a) {
            char b[96];
            std::snprintf(b, sizeof b, f, a...);
            return std::string(b);
        };

        // Geometry: fixed for the life of a run, so the file IS the live value.
        row("row 1",  "screen",          "%d x %d px", scenario_.canvas_px[0], scenario_.canvas_px[1]);
        row("row 3",  "camera",          "%d x %d px", scenario_.resolution[0], scenario_.resolution[1]);
        row("row 4",  "field of view",   "%.1f x %.1f deg", scenario_.fov_deg[0], scenario_.fov_deg[1]);
        row("row 5",  "camera rate",     "%d Hz", scenario_.camera_hz);
        row("row 10", "target size",     "%d px", scenario_.targets.empty() ? 0 : scenario_.targets[0].size_px);
        row("row 13", "max pan",         "%.1f deg/s", scenario_.max_pan_dps);
        row("row 14", "max tilt",        "%.1f deg/s", scenario_.max_tilt_dps);
        row("row 15", "control rate",    "%d Hz", scenario_.control_hz);

        // Damage and disturbance: every one of these is live-adjustable from
        // the controls panel, so every one of them reads the chain.
        const bool live_damage = chain.enabled();
        row_live("row 21", "salt & pepper",
                 live_damage ? fmt("%.0f %%", 100.0 * ln.salt_pepper_p) : std::string("off"),
                 fmt("%.0f %%", 100.0 * scenario_.salt_pepper));
        row_live("row 21", "poisson (shot)",
                 std::string(live_damage && ln.poisson_enabled ? "on" : "off"),
                 std::string(scenario_.noise_poisson ? "on" : "off"));
        row_live("row 22", "read noise",
                 live_damage ? fmt("sigma %.1f", ln.gaussian_sigma) : std::string("off"),
                 fmt("sigma %.0f", scenario_.gaussian_sigma));
        row_live("row 23", "jitter",
                 fmt("%.0f px/frame", pipeline_.source().disturbance().jitter_px_per_frame()),
                 fmt("%.0f px/frame", scenario_.jitter_px_per_frame));
        row_live("row 24", "atmosphere",
                 std::string(live_damage ? atmosphere_name(chain.atmosphere()) : "off"),
                 std::string(atmosphere_name(scenario_.atmosphere)));
        row_live("-",      "defects (hot/dead)",
                 std::string(live_damage && chain.defects_enabled() ? "on" : "off"),
                 std::string("on"));
        {
            const DisturbanceGenerator& d = pipeline_.source().disturbance();
            row_live("row 25", "platform",
                     d.has_platform()
                         ? std::string(d.platform_enabled() ? "on" : "off")
                         : std::string("none"),
                     fmt("%zu component(s)", scenario_.platform.size()));
        }
        row_live("S9.1",   "clutter",
                 fmt("%d sources, %d decoy", live_clutter, live_decoy),
                 fmt("%d sources, %d decoy",
                     scenario_.static_sources, scenario_.decoy_beacons));

        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::TextColored(kMutedCol, "Derived (design S1.4)");
    const auto cam = scenario_.camera_geometry();
    ImGui::Text("IFOV              %.3f urad/px", cam.ifov_urad());
    ImGui::Text("camera authority  %.1f px/frame",
                deg_to_urad(scenario_.max_pan_dps) / cam.ifov_urad() / scenario_.camera_hz);
    ImGui::Text("jitter            %.0f%% of authority",
                100.0 * scenario_.jitter_px_per_frame
                      / (deg_to_urad(scenario_.max_pan_dps) / cam.ifov_urad()
                         / scenario_.camera_hz));
    ImGui::Text("emitters in world %zu", pipeline_.source().emitters().n);

    // §9.4.6's blob table is bounded (design amendment §14.0e). Show the count
    // always, and shout when the bound binds: a truncated frame produces a
    // SHORTER detection list, which is indistinguishable from a clean frame
    // unless something says so.
    const size_t blobs    = pipeline_.perception().last_blob_count();
    const size_t overflow = pipeline_.perception().last_blob_overflow();
    if (overflow > 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "blobs             %zu  (+%zu DROPPED, table full)",
                           blobs, overflow);
    } else {
        ImGui::Text("blobs             %zu / %zu", blobs,
                    ClassicalPerception::kBlobReserve);
    }
    ImGui::End();
}

// ===========================================================================
// run
// ===========================================================================

int Dashboard::run(const Scenario& initial, ScreenshotJob job) {
    job_ = job;
    // `--shot` must still finish. A window you are watching should not.
    continuous_ = job.path.empty();
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

    // A screenshot run creates the window HIDDEN and at a fixed size.
    //
    // Both matter. Hidden, because regenerating the manual's figures should not
    // throw eight windows across whatever the user is doing. Fixed, because a
    // window manager is free to resize a visible window to fit its own idea of
    // the screen — this laptop's tiler gave back 941 x 1130 for a request of
    // 1900 x 1100 — and a figure whose panels are cropped differently on every
    // machine is not a figure, it is a lottery.
    if (!job_.path.empty()) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }
    // 2400 x 1350 for a figure: the right-hand column carries wide tables
    // (the hypothesis table, the compliance matrix, the stage timings) and at
    // 1920 they clip. A figure with a clipped table is a figure that has to be
    // apologised for in its caption.
    const int win_w = job_.path.empty() ? 1900 : 2400;
    const int win_h = job_.path.empty() ? 1100 : 1350;
    window_ = glfwCreateWindow(win_w, win_h, "SAT - Satellite Adaptive Tracker",
                               nullptr, nullptr);
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

    // A screenshot run configures itself before the first frame, so the figure
    // in the manual shows the feature it is a figure OF rather than the default
    // opening state.
    if (!job_.path.empty()) {
        if (job_.imm)          const_cast<Scenario&>(initial).tracking_imm = true;
        if (job_.supervisor)   const_cast<Scenario&>(initial).supervisor_enabled = true;
        if (job_.clutter >= 0) {
            clutter_sources_ = job_.clutter;
            decoy_beacons_   = job_.clutter > 0 ? 1 : 0;
        }
        start_clean_    = !job_.damage;
        beacon_in_view_ = !job_.random_start;
        steps_per_draw_ = 4;   // reach the target frame in a fraction of a second
    }

    rebuild(initial);

    // After rebuild(), because rebuild() constructs the pipeline and the
    // detector is pipeline state. Before the first step, so every frame in the
    // shot ran through the detector the figure claims to be showing.
    if (!job_.path.empty() && job_.strawman) {
        pipeline_.set_detector(DetectorKind::BrightestPixel);
    }

    int64_t shot_countdown = job_.path.empty() ? -1 : job_.after_frames;

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
        draw_hypotheses_panel();
        draw_imm_panel();
        draw_strategy_panel();
        draw_mode_graph();
        draw_scenario_panel();

        ImGui::Render();
        int w = 0, h = 0;
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (redock_frames_ > 0) --redock_frames_;

        // The screenshot is taken AFTER the draw and BEFORE the swap, so the
        // back buffer still holds the frame that was just rendered. Reading
        // after the swap would capture whatever the driver left behind.
        if (shot_countdown >= 0 && frames_ >= shot_countdown && layout_built_
            && redock_frames_ == 0) {
            const bool ok = capture(job_.path);
            glfwSwapBuffers(window_);
            glfwDestroyWindow(window_);
            glfwTerminate();
            return ok ? 0 : 1;
        }

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

// ---------------------------------------------------------------------------
// capture — glReadPixels the window and write it.
//
// OpenCV rather than a hand-rolled PNG writer: it is already a dependency, and
// design §4.2's boundary explicitly sanctions it for "decode, file I/O and test
// oracles". Writing a deflate stream by hand to avoid one imgcodecs link would
// be the wrong kind of purity.
// ---------------------------------------------------------------------------
bool Dashboard::capture(const std::string& path) const {
#if SAT_HAVE_OPENCV
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    if (w <= 0 || h <= 0) return false;

    std::vector<unsigned char> px(static_cast<size_t>(w) * static_cast<size_t>(h) * 3u);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());

    // GL's origin is bottom-left and every image format's is top-left, so the
    // rows come back upside down. Flipped in place rather than by cv::flip so
    // the buffer is only walked once.
    const size_t stride = static_cast<size_t>(w) * 3u;
    for (int y = 0; y < h / 2; ++y) {
        std::swap_ranges(px.begin() + static_cast<ptrdiff_t>(y * stride),
                         px.begin() + static_cast<ptrdiff_t>((y + 1) * stride),
                         px.begin() + static_cast<ptrdiff_t>((h - 1 - y) * stride));
    }

    cv::Mat rgb(h, w, CV_8UC3, px.data(), stride);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    if (!cv::imwrite(path, bgr)) {
        std::fprintf(stderr, "sat-tracker: could not write %s\n", path.c_str());
        return false;
    }
    std::printf("wrote %s  (%d x %d)\n", path.c_str(), w, h);
    return true;
#else
    (void)path;
    std::fprintf(stderr,
        "sat-tracker: --shot needs OpenCV, which this build does not have.\n");
    return false;
#endif
}

int run_dashboard(const Scenario& sc) {
    Dashboard d;
    return d.run(sc);
}

int run_dashboard_screenshot(const Scenario& sc, const ScreenshotJob& job) {
    Dashboard d;
    return d.run(sc, job);
}

}  // namespace sat::gui
