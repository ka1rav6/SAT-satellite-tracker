// gui/dashboard.hpp — the live dashboard (design §12).
//
// §3.1 makes the GUI explicitly graded, under Functional Verification (20%):
// "10-15 min live demo. All mandatory functions - Operational success - GUI".
// So this is a deliverable, not polish.
//
// ---------------------------------------------------------------------------
// WHAT IT IS FOR, WHICH SHAPES WHAT IS IN IT
// ---------------------------------------------------------------------------
// Design §14.1's demo script is the specification for this window. It has to
// support, live and without restarting:
//
//   * showing the camera view with the detection and the truth marked, so a
//     viewer can see the tracker working rather than be told it works;
//   * dialling damage up mid-run — fog, then maximum noise (§14.1, 3:00-5:00);
//   * turning the control loop off and watching the error trace blow up
//     (§14.1, 5:00-6:30), which §12 calls "worth more than any table";
//   * the two graded error traces, plotted separately, because INV-6 says
//     centroiding error and tracking error are different quantities and
//     conflating them makes both meaningless.
//
// ---------------------------------------------------------------------------
// SINGLE-THREADED, THROUGH THE TRIPLE BUFFER
// ---------------------------------------------------------------------------
// The simulation is stepped from inside the render loop rather than on its own
// thread. CP 2.3's property — that a slow display cannot slow the simulation —
// is already built and tested (core/triple_buffer.hpp), and the snapshot is
// still published and acquired through it here, so the seam is exercised.
//
// Running the sim inline buys determinism during a demo: stepping frame by
// frame, pausing exactly where something interesting happens, and knowing that
// what is on screen is precisely the frame the numbers describe. A background
// thread would make the displayed frame and the displayed metrics drift by an
// unpredictable amount, which is the wrong trade for something whose job is to
// be believed.

#pragma once

#include "engine/pipeline.hpp"
#include "metrics/collector.hpp"
#include "gui/gl_texture.hpp"
#include "scenario/scenario.hpp"

#include <deque>
#include <string>
#include <vector>

struct GLFWwindow;

namespace sat::gui {

/// Dock node for each panel, filled by the layout builder.
///
/// The panels are docked with an explicit SetNextWindowDockID before each
/// Begin, rather than by DockBuilderDockWindow alone. DockBuilderDockWindow
/// records the assignment in ImGui's window SETTINGS, which a window reads when
/// it is first created — a path that did not take effect here (the layout built
/// correctly, with the right node IDs and a 1600x950 work area, and every panel
/// still came up floating and stacked at the origin). Setting the dock ID
/// directly is explicit, needs no settings round-trip, and works on frame one.
struct DockIds {
    unsigned int left = 0, left_bottom = 0, centre = 0, centre_bottom = 0;
    unsigned int right = 0, right_mid = 0, right_bottom = 0;
};

/// A rolling window of a scalar, for the live plots. Fixed capacity so the
/// plots cannot grow without bound over a long demo.
class Trace {
public:
    explicit Trace(size_t capacity = 4000) : cap_(capacity) {}

    void push(double x, double y) {
        xs_.push_back(x);
        ys_.push_back(y);
        if (xs_.size() > cap_) { xs_.pop_front(); ys_.pop_front(); }
        if (y > 0.0) { sum_sq_ += y * y; ++n_; }
    }
    void clear() { xs_.clear(); ys_.clear(); sum_sq_ = 0.0; n_ = 0; }

    /// ImPlot wants contiguous arrays, and a deque is not. Flattened on demand
    /// into a reused buffer rather than kept contiguous, because a ring buffer
    /// that also had to be contiguous would need a memmove per sample.
    void flatten(std::vector<double>& x, std::vector<double>& y) const {
        x.assign(xs_.begin(), xs_.end());
        y.assign(ys_.begin(), ys_.end());
    }

    /// Running RMSE, which is the figure design §12 asks to be shown live
    /// beside the centroiding-error plot.
    [[nodiscard]] double rmse() const {
        return n_ ? std::sqrt(sum_sq_ / static_cast<double>(n_)) : 0.0;
    }
    [[nodiscard]] size_t count() const noexcept { return n_; }
    [[nodiscard]] bool empty() const noexcept { return xs_.empty(); }

private:
    std::deque<double> xs_, ys_;
    size_t cap_;
    double sum_sq_ = 0.0;
    size_t n_ = 0;
};

// -----------------------------------------------------------------------
// ScreenshotJob — run for a while, save the window, exit.
//
// The docs are supposed to show what the product looks like, and a
// screenshot pasted into a repository is a claim nobody can check: it rots
// silently the moment a panel moves. This makes every figure in the manual
// the output of a COMMAND — `just screenshots` regenerates all of them —
// so a stale one is a diff rather than a surprise.
//
// It is also the only automated check that the GUI still starts and draws.
// CP 15.0's acceptance is "a window opens and renders at >= 60 FPS", which
// nothing in CI could verify before.
// -----------------------------------------------------------------------
struct ScreenshotJob {
    std::string path;          ///< PNG to write; empty means interactive
    int  after_frames = 90;    ///< simulation frames to run first
    bool imm          = false; ///< force the IMM on before running
    bool supervisor   = false; ///< force the supervisor on
    int  clutter      = -1;    ///< >= 0 overrides the clutter count
    bool damage       = false; ///< start with the damage chain on
    bool random_start = false; ///< honour spec row 11 instead of centring
    /// Run CP 4.11's straw man (brightest pixel) instead of §9.4's detector.
    /// The guide's ablation figure has to SHOW the straw man selected, which
    /// means the shot has to switch it rather than describe it.
    bool strawman     = false;
    /// Panel to bring to the front of its tab bar before the shot is taken.
    /// Docked panels share a tab bar, so a figure OF a panel has to ask for it.
    std::string focus;
};

// ---------------------------------------------------------------------------
// Dashboard
// ---------------------------------------------------------------------------
class Dashboard {
public:
    /// Open the window and run until the user closes it, or — when `job.path`
    /// is set — until the screenshot has been taken.
    /// Returns a process exit code.
    [[nodiscard]] int run(const Scenario& initial, ScreenshotJob job = {});

private:
    /// glReadPixels the whole window, flip it, and write it. Returns false and
    /// explains on stderr if the image could not be written.
    [[nodiscard]] bool capture(const std::string& path) const;

    ScreenshotJob job_{};
    void step_simulation();
    void rebuild(const Scenario& sc);

    void draw_menu_bar();
    void draw_controls();
    void draw_camera_view();
    void draw_screen_overview();
    void draw_error_plots();
    void draw_metrics();
    void draw_tracking_panel();
    void draw_scenario_panel();

    // --- CP 15.1's remaining §12 panels ------------------------------------
    /// The IMM's three mode probabilities over time, and the turn rate it is
    /// estimating. The panel CP 10.5 is written against: "the mode plot shows
    /// the shift" at a figure-8 crossing.
    void draw_imm_panel();
    /// §10.6's strategy timeline — what the supervisor chose, when, and what
    /// condition made it choose. CP 12.2: "switches visible on a timeline".
    void draw_strategy_panel();
    /// §10.4's mode FSM, drawn as a graph with the live state highlighted.
    /// CP 6.6's acceptance criterion before §14.0 replaced it with a trace.
    void draw_mode_graph();
    /// §10.2's priority policy: every live hypothesis, its score, and why the
    /// committed one is committed. Without this the tracker's most important
    /// decision is invisible.
    void draw_hypotheses_panel();
    /// Bring a docked panel to the front of its tab bar, for a screenshot run.
    void focus_if_requested(const char* title);

    GLFWwindow* window_ = nullptr;

    Scenario  scenario_{};
    Pipeline  pipeline_{};
    GlTexture camera_tex_{};

    // --- run state ---------------------------------------------------------
    bool   layout_built_   = false;
    /// Viewport size seen last frame, and how many frames it has been steady.
    /// The layout waits for the window manager to stop resizing us.
    struct { float x = 0.0f, y = 0.0f; } layout_size_{};
    int    stable_frames_  = 0;
    int    redock_frames_  = 0;

    /// Docking condition for the panels: Always for a few frames after a
    /// layout rebuild (which undocks everything), Once otherwise so a panel a
    /// user has dragged somewhere stays where they put it.
    [[nodiscard]] int dock_cond() const noexcept;

    DockIds dock_{};

    /// Place the beacon inside the field of view at t = 0, overriding spec
    /// row 11's "random" default.
    ///
    /// On by default in the dashboard, and it needs justifying. Row 11's random
    /// placement is correct and the headless runs honour it — but the camera
    /// sees 7.68% of the screen (design §1.4), so a randomly placed beacon
    /// starts off-screen 92% of the time, and ACQUIRING it is the search
    /// problem, which is Stage 13. Until then, opening the dashboard on a
    /// random scenario shows a camera hunting for something it cannot find,
    /// which says nothing about the parts that are built.
    ///
    /// The checkbox makes the override explicit rather than hidden, and turning
    /// it off is itself informative: it shows exactly why Stage 13 is needed.
    bool   beacon_in_view_  = true;

    /// Open with the damage chain quiet. See rebuild() for why.
    bool   start_clean_     = true;

    /// Clutter counts, applied on rebuild. Zero at startup for the same reason
    /// the damage chain starts quiet: §9.1 makes some clutter brighter than the
    /// beacon, so the straw-man detector locks onto it immediately and the
    /// dashboard would open on a failure.
    int    clutter_sources_ = 0;
    int    decoy_beacons_   = 0;
    bool   running_        = true;
    bool   step_once_      = false;
    bool   finished_       = false;
    /// Interactive dashboard: ignore the scenario's duration and run until
    /// Pause. Screenshot jobs leave this false so they still end on schedule.
    bool   continuous_     = true;
    int    steps_per_draw_ = 1;     ///< simulation frames per rendered frame

    // --- live traces -------------------------------------------------------
    Trace centroid_image_{};
    Trace centroid_screen_{};
    Trace tracking_{};
    Trace saturation_{};

    /// The IMM's three mode probabilities, and the priority policy's two
    /// scores. Same rolling window as the error traces.
    Trace imm_cv_{}, imm_ca_{}, imm_ct_{};
    Trace score_committed_{}, score_rival_{};

    /// One entry per supervisor switch, for the strategy timeline. Bounded:
    /// §10.6's dwell allows at most one switch per second, so a twenty-minute
    /// demo cannot exceed this.
    struct StrategyMark {
        double      t_s = 0.0;
        std::string label;      ///< what changed, already formatted
        float       snr = 0.0f;     ///< the condition that triggered it
        float       q_scale = 1.0f; ///< and what it did to the filter
    };
    std::vector<StrategyMark> strategy_marks_;

    // The beacon's true path and the detector's reported path, in screen
    // coordinates, for the overview panel. Subsampled: a 120 s run at 30 Hz is
    // 3600 points, which is more than a 400-pixel-wide panel can show.
    std::vector<double> truth_path_x_, truth_path_y_;
    std::vector<double> det_path_x_, det_path_y_;

    // Scratch for ImPlot, reused so drawing allocates nothing steady-state.
    std::vector<double> plot_x_, plot_y_, plot_y2_;

    // -----------------------------------------------------------------------
    // THE COMPLIANCE PANEL USES THE SAME COLLECTOR AS EVERYTHING ELSE — P1-8.
    //
    // It used to keep its own counters and compute its own ratios, in parallel
    // with metrics/collector.cpp. Two implementations of §13.1, and they
    // disagreed the moment either moved:
    //
    //   * the panel graded row 17 on the WHOLE-RUN tracking RMS, which is
    //     transient-dominated. Sixty frames into a run it read "67.39 px" in
    //     red against a summary that said 16.94 px. Both were right about
    //     different things and the screen said neither.
    //   * it graded row 18 on `frames_detected / frames_in_fov`, which is not
    //     retention — it is detection rate. The graded figure is held frames
    //     over post-acquisition frames (P0-2), and the panel had no notion of
    //     post-acquisition at all.
    //
    // A dashboard that disagrees with the artifacts it writes is worse than
    // one that reports less: the numbers a judge reads off the screen are the
    // ones they will quote back. So the panel now feeds the real collector and
    // reads the real RunMetrics, and there is one definition of every row.
    //
    // `metrics_dirty_` avoids calling finish() once per rendered frame — it
    // sorts several sample vectors — so it recomputes only when a simulation
    // frame has actually been added.
    // -----------------------------------------------------------------------
    MetricCollector metrics_{};
    RunMetrics      metrics_snapshot_{};
    bool            metrics_dirty_ = true;

    /// Recompute metrics_snapshot_ if a frame has been added since the last
    /// call. Cheap when clean.
    const RunMetrics& live_metrics();

    // --- counters ----------------------------------------------------------
    int64_t frames_          = 0;
    int64_t frames_detected_ = 0;
    int64_t frames_in_fov_   = 0;
    int64_t false_alarms_    = 0;
    double  worst_tracking_  = 0.0;

    std::string status_;
};

/// Entry point for `sat-tracker --gui`.
[[nodiscard]] int run_dashboard(const Scenario& sc);

/// Entry point for `sat-tracker --gui --shot FILE`. See Dashboard::ScreenshotJob.
[[nodiscard]] int run_dashboard_screenshot(const Scenario& sc,
                                           const ScreenshotJob& job);

}  // namespace sat::gui
