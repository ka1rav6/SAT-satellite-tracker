// engine/pipeline.cpp

#include "engine/pipeline.hpp"

#include <cmath>

namespace sat {

void Pipeline::build(const PipelineConfig& cfg, EmitterSoA emitters) {
    cfg_ = cfg;
    source_.build(cfg_.synthetic, std::move(emitters));
    gimbal_.reset(cfg_.pan, cfg_.tilt, cfg_.initial_boresight);
    control_.reset(cfg_.gains);
    cmd_rate_ = Rate2{};
    frame_    = 0;
    last_     = FrameRecord{};
}

bool Pipeline::step() {
    SAT_ZONE(timers_, Stage::FrameTotal);

    const Clock& clk = source_.clock();
    const double truth_dt = clk.truth_dt();
    const int    subticks = clk.camera_divisor();

    // -----------------------------------------------------------------------
    // B1-B3: sub-tick loop. The world and the plant run at 300 Hz while frames
    // arrive at 30. Design §6.2 makes this explicit, and it matters: a gimbal
    // integrated once per frame at 33 ms would misrepresent its own dynamics,
    // and the acceleration limit in particular would be unobservable.
    // -----------------------------------------------------------------------
    for (int s = 0; s < subticks; ++s) {
        {
            SAT_ZONE(timers_, Stage::WorldAdvance);
            source_.advance_world(truth_dt);
        }
        {
            SAT_ZONE(timers_, Stage::GimbalStep);
            // ===============================================================
            // INV-2 LIVES HERE. This is the line that closes the loop: the
            // rate the controller produced at the end of the LAST frame is what
            // moves the mount before THIS frame is rendered.
            //
            // With control_enabled false the mount is commanded to hold still,
            // which is what CP 1.8 (b) checks — the camera must then fail to
            // follow, proving the following was caused by our controller and
            // not by the simulator conveniently centring the target.
            // ===============================================================
            const Rate2 applied = cfg_.control_enabled ? cmd_rate_ : Rate2{};
            gimbal_.step(applied, truth_dt);
        }
    }

    // -----------------------------------------------------------------------
    // B4: acquire. The source renders at the true boresight; we hand it the
    // commanded one and it adds the disturbance itself.
    // -----------------------------------------------------------------------
    const Angle2 commanded = gimbal_.true_position();
    SourceFrame frame;
    {
        SAT_ZONE(timers_, Stage::FrameAcquire);
        if (!source_.next(commanded, frame)) return false;
    }

    FrameRecord rec{};
    rec.frame          = frame_;
    rec.time_s         = frame.timestamp_s;
    rec.boresight_cmd  = commanded;
    rec.boresight_true = frame.truth.boresight_true;
    rec.cmd_rate       = cmd_rate_;

    // -----------------------------------------------------------------------
    // B5-B15: perception.
    //
    // Note what is passed: frame.pixels and nothing else. The detector cannot
    // reach frame.truth even though it is sitting right there in the same
    // struct, because it is never handed the enclosing SourceFrame. That is
    // INV-1 enforced at the call site, on top of the link-level enforcement in
    // cmake/modules.cmake.
    // -----------------------------------------------------------------------
    SimpleDetection det;
    {
        SAT_ZONE(timers_, Stage::Centroid);
        det = detect_brightest_subpixel(frame.pixels, frame.width, frame.height,
                                        cfg_.detector_window, cfg_.detector_floor);
    }

    // -----------------------------------------------------------------------
    // B16: image pixels -> world angle, via the COMMANDED boresight.
    //
    // Commanded, not true: the tracker does not know the true one. Using truth
    // here would silently cancel the pointing error and make the whole system
    // look perfect for the wrong reason — the single easiest way to accidentally
    // cheat in this project.
    // -----------------------------------------------------------------------
    Angle2 aim = commanded;
    if (det.found) {
        rec.detected        = true;
        rec.detection_img   = det.centre;
        rec.detection_peak  = det.peak;

        const Angle2 offset = cfg_.synthetic.camera.unproject(det.centre);
        const Angle2 world  = commanded + offset;
        rec.detection_screen = cfg_.synthetic.screen.to_pixel(world);
        aim = world;
    }

    // -----------------------------------------------------------------------
    // B22-B27: control.
    // -----------------------------------------------------------------------
    {
        SAT_ZONE(timers_, Stage::Control);
        const Angle2 measured = gimbal_.position();   // the QUANTISED encoder
        const bool az_sat = std::fabs(gimbal_.az().rate())
                          >= gimbal_.az().params().max_rate_urad_s * 0.999;
        const bool el_sat = std::fabs(gimbal_.el().rate())
                          >= gimbal_.el().params().max_rate_urad_s * 0.999;

        if (det.found) {
            cmd_rate_ = control_.compute(aim, measured,
                                         Rate2{}, Rate2{},   // FF arrives at CP 10.1
                                         source_.clock().control_dt(),
                                         az_sat, el_sat);
        } else {
            // INV-9's spirit: with no detection there is nothing to aim at, and
            // inventing a command would be worse than holding. Real coasting on
            // a filter prediction arrives at CP 6.4.
            cmd_rate_ = Rate2{};
        }
    }

    // -----------------------------------------------------------------------
    // B28: metrics. THE ONLY PLACE TRUTH IS READ.
    //
    // INV-6: centroiding error and tracking error are different quantities and
    // are computed, stored and reported separately. Conflating them would make
    // both meaningless — one measures the detector, the other the control loop.
    // -----------------------------------------------------------------------
    {
        SAT_ZONE(timers_, Stage::Metrics);
        if (const FrameTruth::Target* t = frame.truth.primary()) {
            rec.truth_valid  = true;
            rec.truth_screen = t->screen_pos;
            rec.truth_in_fov = t->in_fov;

            // Centroiding error: how well we located the beacon in the image.
            // Computed ONLY on frames with a detection (design §13.1).
            if (rec.detected) {
                rec.centroid_error_px = (rec.detection_screen - t->screen_pos).norm();
            }
            // Tracking error: how well the mount is pointed at the beacon.
            // Independent of whether we detected it this frame.
            const Pixel2 bore_screen = cfg_.synthetic.screen.to_pixel(rec.boresight_true);
            rec.tracking_error_px = (bore_screen - t->screen_pos).norm();
        }
    }

    last_ = rec;
    ++frame_;
    return true;
}

void Pipeline::run(std::vector<FrameRecord>& out) {
    out.clear();
    while (step()) out.push_back(last_);
}

}  // namespace sat
