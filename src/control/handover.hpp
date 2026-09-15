// control/handover.hpp — CP 10.7's handover stub.
//
// ---------------------------------------------------------------------------
// WHAT HANDOVER IS, AND WHY AN 80-LINE STUB IS THE RIGHT AMOUNT
// ---------------------------------------------------------------------------
// The problem statement asks for COARSE alignment. In a real FSOC terminal that
// is the first half of a two-stage PAT chain: a wide-field camera walks the
// beam into a narrow capture range, and a fine sensor — typically a quadrant
// photodiode or a position-sensitive detector behind the receive aperture —
// takes over and closes a much faster, much tighter loop on a fast steering
// mirror.
//
// Building the fine loop is out of scope and would be a different project. But
// a coarse tracker that never says "I am done, you may take over" has not
// finished the job it was given: the deliverable of coarse alignment is a
// HANDOVER, and without one there is no definition of success beyond "the error
// looks small". §10.4 asks for the stub for exactly that reason — it completes
// the narrative the problem statement only implies — and it is a stub because
// what matters is the CRITERION, not the fine loop behind it.
//
// ---------------------------------------------------------------------------
// THE CRITERION, AND WHY IT IS NOT THE TRACKING ERROR
// ---------------------------------------------------------------------------
// §10.4: "Track -> Handover: RMS error < capture_range/3 for 30 consecutive
// frames." Three things in that sentence are load-bearing.
//
//   capture_range   ~1 mrad, the angular window in which a quad cell produces
//                   a usable error signal at all. At the default 109 urad/px
//                   that is 9.2 px — the whole handover decision lives inside
//                   a box a tenth the width of one beacon's search step.
//
//   / 3             handing over AT the capture range is handing over to a
//                   sensor that is about to lose the beam. A third leaves
//                   margin for the transient the switch itself causes.
//
//   30 consecutive  one quiet frame proves nothing; a second of them is
//                   evidence the loop is actually settled rather than passing
//                   through zero.
//
// The quantity fed in is NOT the tracking error from the metrics. That number
// is computed against truth, and a mode FSM that reads truth would be deciding
// to hand over using information the real system does not have — the same
// conflation INV-6 exists to prevent.
//
// What is used instead is genuinely observable: the beacon's offset from the
// boresight IN THE IMAGE. The frame is rendered at the true boresight, so
// `detection_image - image_centre` IS the angular offset a co-boresighted quad
// cell would see, and it costs nothing to compute. That is the honest reading
// of what a fine sensor measures, and it is why this file needs neither truth
// nor the world.

#pragma once

#include "core/ring.hpp"
#include "core/units.hpp"

#include <cmath>
#include <cstddef>

namespace sat {

// ---------------------------------------------------------------------------
// QuadrantDetector — the fine sensor, modelled only as far as the decision
// needs.
//
// A quad cell splits the spot across four elements and reports the normalised
// difference between them. For a spot of half-width w at offset d the response
// is approximately d/w near the centre and saturates once the spot leaves one
// quadrant entirely — it is a NULL sensor, precise at zero and useless far from
// it, which is exactly why it needs a coarse stage to hand it a beam.
//
// The saturating response is modelled because it is the part that matters: it
// is what makes "inside capture range" a meaningful state rather than a
// threshold on a number that keeps growing.
// ---------------------------------------------------------------------------
struct QuadrantParams {
    /// The angular window in which the cell produces a usable signal.
    double capture_urad = 1000.0;
    /// The linear region, inside which the response is proportional to offset.
    /// A real cell's linear range is roughly the spot size; a tenth of the
    /// capture range is a conservative stand-in.
    double linear_urad = 100.0;
};

class QuadrantDetector {
public:
    void reset(const QuadrantParams& p) noexcept { p_ = p; }

    /// True when the beam is inside the window where the cell has signal.
    [[nodiscard]] bool in_capture(Angle2 offset_urad) const noexcept {
        return std::hypot(offset_urad.x, offset_urad.y) <= p_.capture_urad;
    }

    /// The normalised error signal, each component in [-1, 1]. Zero outside the
    /// capture range — not "large", ZERO, because a null sensor with no beam on
    /// it reports no error rather than a big one. Treating loss of signal as a
    /// large error is how a fine loop runs away.
    [[nodiscard]] Angle2 signal(Angle2 offset_urad) const noexcept {
        if (!in_capture(offset_urad)) return Angle2{};
        const double s = p_.linear_urad > 0.0 ? 1.0 / p_.linear_urad : 0.0;
        return Angle2{clamp_abs(offset_urad.x * s, 1.0),
                      clamp_abs(offset_urad.y * s, 1.0)};
    }

    [[nodiscard]] const QuadrantParams& params() const noexcept { return p_; }

private:
    QuadrantParams p_{};
};

// ---------------------------------------------------------------------------
// HandoverMonitor — the running RMS the FSM's transition is written against.
//
// A fixed-capacity ring, INV-4: the window is 30 frames by design and a
// std::deque here would allocate once per run for no reason. The ring is sized
// to the largest window anyone would configure rather than to the default, so
// changing handover_frames is a parameter change and not a recompile of this
// file's assumptions.
// ---------------------------------------------------------------------------
class HandoverMonitor {
public:
    static constexpr size_t kCapacity = 64;

    void reset(int window_frames) noexcept {
        window_ = static_cast<size_t>(window_frames <= 0 ? 1 : window_frames);
        if (window_ > kCapacity) window_ = kCapacity;
        hist_.fill(0.0);
        n_ = 0;
    }

    /// Feed one frame's observed offset magnitude, microradians. Frames with no
    /// detection feed nothing — INV-9's principle: a missing measurement is a
    /// gap, not a zero, and a zero here would be the most flattering possible
    /// value for the quantity being tested.
    void push(double offset_urad) noexcept {
        hist_.push(offset_urad);
        if (n_ < window_) ++n_;
    }

    /// Drop the history. Called whenever the loop stops tracking, so that a
    /// handover decision is never made partly on evidence from before a
    /// dropout.
    void clear() noexcept { hist_.fill(0.0); n_ = 0; }

    /// RMS over the window, or a large number until the window is full. Large
    /// rather than zero for the same reason as above: an unfilled window must
    /// not read as a settled loop.
    [[nodiscard]] double rms_urad() const noexcept {
        if (n_ < window_) return 1e9;
        double sum_sq = 0.0;
        for (size_t k = 0; k < window_; ++k) {
            const double v = hist_.at_back(k);
            sum_sq += v * v;
        }
        return std::sqrt(sum_sq / static_cast<double>(window_));
    }

    [[nodiscard]] size_t filled() const noexcept { return n_; }

private:
    Ring<double, kCapacity> hist_{};
    size_t                  window_ = 30;
    size_t                  n_      = 0;
};

}  // namespace sat
