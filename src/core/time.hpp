// core/time.hpp — the simulation clock.
//
// INV-3 (design §2) forbids reading a wall clock anywhere in the simulation
// path. Every notion of "when" in this program comes from this file, and it is
// derived from an integer tick count:
//
//      seconds = tick / truth_hz
//
// NOT from an accumulated `t += dt`. Accumulation drifts: adding 1/300 three
// hundred times does not give exactly 1.0, and after a two-minute run the error
// is large enough to move a fast target by a measurable fraction of a pixel.
// Dividing an exact integer by an exact integer gives the same double on every
// machine, every time, which is what bit-exact reproducibility requires.
//
// The rate structure (design §6.2) is:
//
//      truth_hz   300 Hz   the world and the disturbances advance this fast
//      camera_hz   30 Hz   spec row 5, minimum 30 -- a frame every 10 ticks
//      control_hz  30 Hz   spec row 15, minimum 20
//
// truth_hz must be an exact integer multiple of both, so that a camera frame
// and a control update always land exactly on a truth tick. A non-integer
// ratio would mean interpolating the world at frame time, which is both slower
// and a source of irreproducibility.

#pragma once

#include "core/result.hpp"

#include <cstdint>
#include <string>

namespace sat {

// ---------------------------------------------------------------------------
// Clock — integer tick counter plus the derived divisors.
//
// Deliberately has no wall-clock member of any kind. If you find yourself
// wanting one here, what you actually want is core/profile.hpp, which is
// outside the simulation path and cannot affect results.
// ---------------------------------------------------------------------------
class Clock {
public:
    Clock() = default;

    // -----------------------------------------------------------------------
    // Validating factory (CP 0.3 acceptance: rejects truth_hz % camera_hz != 0).
    //
    // Returns a Result rather than throwing because scenario loading reports
    // every problem it can find with a spec-row citation (design §7.5), and a
    // thrown exception loses that context.
    // -----------------------------------------------------------------------
    [[nodiscard]] static Result<Clock> make(int truth_hz, int camera_hz, int control_hz) {
        if (truth_hz <= 0) {
            return Err("sim.truth_hz must be positive, got " + std::to_string(truth_hz));
        }
        if (camera_hz < 30) {
            // Spec row 5: "Camera update rate >= 30 Hz". This is a hard floor,
            // not a recommendation, so we refuse rather than warn.
            return Err("sim.camera_hz = " + std::to_string(camera_hz) +
                       " is below the required minimum of 30 Hz (specification row 5)");
        }
        if (control_hz < 20) {
            // Spec row 15: "Update interval >= 20 Hz".
            return Err("sim.control_hz = " + std::to_string(control_hz) +
                       " is below the required minimum of 20 Hz (specification row 15)");
        }
        if (truth_hz % camera_hz != 0) {
            return Err("sim.truth_hz = " + std::to_string(truth_hz) +
                       " must be an exact multiple of sim.camera_hz = " +
                       std::to_string(camera_hz) +
                       " so that camera frames land on truth ticks (INV-3)");
        }
        if (truth_hz % control_hz != 0) {
            return Err("sim.truth_hz = " + std::to_string(truth_hz) +
                       " must be an exact multiple of sim.control_hz = " +
                       std::to_string(control_hz) +
                       " so that control updates land on truth ticks (INV-3)");
        }

        Clock c;
        c.truth_hz_        = truth_hz;
        c.camera_hz_       = camera_hz;
        c.control_hz_      = control_hz;
        c.camera_divisor_  = truth_hz / camera_hz;
        c.control_divisor_ = truth_hz / control_hz;
        c.truth_dt_        = 1.0 / static_cast<double>(truth_hz);
        c.camera_dt_       = 1.0 / static_cast<double>(camera_hz);
        c.control_dt_      = 1.0 / static_cast<double>(control_hz);
        return Ok(c);
    }

    // --- advancing ---------------------------------------------------------

    /// Advance by exactly one truth tick. The only way time moves.
    void tick() noexcept { ++tick_; }

    /// Rewind to t = 0 without losing the configured rates. Used between sweep
    /// runs inside one process, where re-deriving the divisors would be waste.
    void reset() noexcept { tick_ = 0; }

    // --- queries -----------------------------------------------------------

    [[nodiscard]] int64_t tick_index() const noexcept { return tick_; }

    /// Current simulation time. Exact integer division, never accumulated.
    [[nodiscard]] double seconds() const noexcept {
        return static_cast<double>(tick_) / static_cast<double>(truth_hz_);
    }

    /// Time at an arbitrary tick. Handy for the motion-blur substep loop, which
    /// needs to evaluate the world at fractional positions between two ticks.
    [[nodiscard]] double seconds_at(int64_t t) const noexcept {
        return static_cast<double>(t) / static_cast<double>(truth_hz_);
    }

    /// True on the ticks where a camera frame is produced. Tick 0 is a frame:
    /// the run starts by looking at the world, not by waiting 33 ms.
    [[nodiscard]] bool is_camera_tick() const noexcept { return tick_ % camera_divisor_ == 0; }

    /// True on the ticks where the controller runs.
    [[nodiscard]] bool is_control_tick() const noexcept { return tick_ % control_divisor_ == 0; }

    /// How many camera frames have been produced up to and including now.
    [[nodiscard]] int64_t frame_index() const noexcept { return tick_ / camera_divisor_; }

    [[nodiscard]] int    truth_hz()        const noexcept { return truth_hz_; }
    [[nodiscard]] int    camera_hz()       const noexcept { return camera_hz_; }
    [[nodiscard]] int    control_hz()      const noexcept { return control_hz_; }
    [[nodiscard]] int    camera_divisor()  const noexcept { return camera_divisor_; }
    [[nodiscard]] int    control_divisor() const noexcept { return control_divisor_; }

    /// The three fixed timesteps. Constants, not measurements.
    [[nodiscard]] double truth_dt()   const noexcept { return truth_dt_; }
    [[nodiscard]] double camera_dt()  const noexcept { return camera_dt_; }
    [[nodiscard]] double control_dt() const noexcept { return control_dt_; }

    /// Total truth ticks in a run of the given length. Rounded to whole ticks
    /// so that two runs of the same configured duration always execute exactly
    /// the same number of steps.
    [[nodiscard]] int64_t ticks_for(double duration_s) const noexcept {
        return static_cast<int64_t>(duration_s * static_cast<double>(truth_hz_) + 0.5);
    }

private:
    int64_t tick_            = 0;

    int     truth_hz_        = 300;
    int     camera_hz_       = 30;
    int     control_hz_      = 30;
    int     camera_divisor_  = 10;
    int     control_divisor_ = 10;

    double  truth_dt_        = 1.0 / 300.0;
    double  camera_dt_       = 1.0 / 30.0;
    double  control_dt_      = 1.0 / 30.0;
};

}  // namespace sat
