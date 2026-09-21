// engine/deadline.hpp — P1-10: what happens when a frame takes too long.
//
// THE QUESTION THIS ANSWERS
//
// "What does your system do when processing is slower than the camera?" The
// honest answer for this project used to be "the simulation slows down with
// it", because the loop is a synchronous pull: source_.next() hands over the
// next frame whenever it is asked, and the simulated clock advances by exactly
// one camera period regardless of how long the previous frame took. On a
// virtual testbed that is a defensible choice — but "it cannot happen by
// construction" is not a real-time argument, it is the absence of one, and a
// coarse-alignment loop that silently falls behind its sensor is a loop that
// is pointing at where the target used to be.
//
// THE DESIGN PROBLEM, AND THE ONE DECISION THAT SHAPES THIS FILE
//
// A deadline policy driven by the wall clock is NOT REPRODUCIBLE. Two runs of
// the same scenario on the same machine will shed on different frames, produce
// different detections and end with different digests. INV-3 — bit-exact
// reproducibility — is one of the nine invariants, and it is the one that the
// whole evidence base rests on: every measurement in docs/RESULTS.md is
// quotable only because the run that produced it can be run again.
//
// So the two halves are SEPARATED, and this file is only the first of them:
//
//   THE POLICY (here).   Given a sequence of frame costs and a budget, decide
//                        which frames missed and what to shed next. A pure
//                        function of its inputs. No clock, no syscall, no
//                        global state. Completely deterministic and therefore
//                        completely testable.
//
//   THE COST SOURCE      Where those numbers come from. Either the real wall
//   (engine/pipeline).   clock (--realtime, for a demo, explicitly NOT
//                        reproducible and reported as such) or a deterministic
//                        injected schedule (--inject-stall, for tests and for
//                        CI, reproducible to the bit).
//
// That split is what lets a test assert "a 50 ms stall produces exactly one
// deadline miss, sheds for exactly four frames, and re-acquires" and have the
// assertion mean something. A test written against the wall clock could only
// ever assert that SOMETHING happened.

#pragma once

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// ShedDecision — what the detector is asked to give up on the next frame.
// ---------------------------------------------------------------------------
// The ladder is ordered by how much accuracy each rung costs, cheapest first,
// because shedding is a bet that a slightly worse answer now beats a better
// answer after the target has moved. The measured savings are from the stage
// table on scenarios/spec_defaults.toml, 640x480, full-frame pass:
//
//   rung 1  skip the 3x3 median          -635 us   costs impulse rejection,
//                                                  which row 21's salt-and-
//                                                  pepper noise needs — so
//                                                  this is first only because
//                                                  it is cheapest to undo
//   rung 2  halve the window floor       -~400 us  costs the CFAR annulus some
//                                                  of its training ring
// THERE IS NO THIRD RUNG, AND THERE WAS.
//
// The ladder originally had one: fall back to §9.4's straw-man detector, worth
// about 17 ms. It was removed after it was measured, and the measurement is
// worth keeping because the conclusion is not obvious.
//
// When shedding actually REDUCES the frame's cost — the ordinary case — the
// ladder never needs a third rung. Measured on spec_defaults over 20 s with a
// 4 ms wall-clock budget against a 3.06 ms p50, the first two rungs bring the
// frame back inside budget, the ladder settles, 597 frames run degraded, and
// retention stays at 99.67% with row 18's target loss at 0.00%.
//
// When shedding CANNOT reduce the cost — an external stall, CPU contention,
// anything the detector's own workload does not control — the ladder pins at
// its top rung for as long as the load lasts. With the straw man on that rung
// that meant running the straw man for hundreds of consecutive frames, and
// the straw man locks onto the brightest thing in the frame. Retention fell to
// 26.7%. The system gave up the target to keep the frame rate, which is the
// wrong trade in every scenario this project has: a coarse-alignment loop that
// has lost the beacon is not degraded, it has failed.
//
// So the automatic ladder stops at rung 2. Both remaining rungs degrade how
// WELL the detector sees; neither changes WHAT it is looking for, so neither
// can walk the track onto a different object. The straw man still exists and
// is still runnable — it is §9.4's ablation arm, reachable through
// PipelineConfig::Detector — but it is an experiment a person chooses, never
// a rung a governor escalates into. §9.4 already said it "must never be the
// default"; this is the same rule applied to the default's failure path.
struct ShedDecision {
    bool skip_median = false;
    bool shrink_roi  = false;
    /// 0 = nothing shed. Monotone: each level implies every level below it.
    int  level       = 0;

    [[nodiscard]] constexpr bool any() const noexcept { return level > 0; }
};

// ---------------------------------------------------------------------------
// DeadlineGovernor — the policy.
// ---------------------------------------------------------------------------
class DeadlineGovernor {
public:
    /// The highest rung the ladder goes to. Two — see ShedDecision above for
    /// the third rung that was measured, found to trade the target for the
    /// frame rate, and removed.
    static constexpr int kMaxLevel = 2;

    /// How many consecutive comfortable frames it takes to climb back down one
    /// rung. Asymmetric on purpose: shedding reacts in ONE frame because the
    /// deadline has already been missed and the next one is about to be, while
    /// recovery waits, because a system that un-sheds the moment it fits will
    /// oscillate between rungs and spend half its frames in the expensive
    /// configuration it just proved it could not afford.
    static constexpr int kRecoverFrames = 8;

    /// A frame must come in under this FRACTION of budget to count as
    /// comfortable. Recovering at exactly 100% would mean recovering into the
    /// configuration that just missed; the margin is what makes the ladder
    /// settle instead of ring.
    static constexpr double kRecoverFraction = 0.70;

    // -----------------------------------------------------------------------
    /// `budget_us` is the wall-clock time one frame is allowed. The natural
    /// value is the camera period — at 30 Hz, 33,333 us — because that is the
    /// rate at which the next frame arrives whether or not this one is done.
    /// `enabled` false makes every call a no-op and the governor invisible.
    void configure(double budget_us, bool enabled) noexcept {
        budget_us_ = budget_us > 0.0 ? budget_us : 0.0;
        enabled_   = enabled && budget_us_ > 0.0;
        reset();
    }

    void reset() noexcept {
        level_ = 0;
        comfortable_run_ = 0;
        misses_ = 0;
        shed_frames_ = 0;
        last_missed_ = false;
        worst_overrun_us_ = 0.0;
    }

    // -----------------------------------------------------------------------
    /// Report what the frame just finished actually cost. This is the ONLY
    /// input to the policy, and it is deliberately a plain number: the
    /// governor neither knows nor cares whether it came from a clock or from a
    /// test's injected schedule.
    /// `may_shed` is the caller's statement that there is something worth
    /// protecting — in practice, that the track is confirmed.
    ///
    /// WHY THIS PARAMETER EXISTS, because it was not in the first version and
    /// the first version collapsed every run it was enabled on.
    ///
    /// Shedding during SEARCH is self-defeating. A frame with no confirmed
    /// track gets the whole frame (detect_window rule 1) and legitimately
    /// costs 19.0 ms on 640x480 — that is not a fault condition, it is what
    /// searching costs. Charging those frames against the deadline climbed the
    /// ladder to the straw man within three frames of startup, before any
    /// track had ever confirmed, and the straw man cannot acquire in clutter.
    /// The run then never acquired at all: retention fell from 99.67% to
    /// 20.00% and row 18's target loss from 0.00% to 98.99%, IDENTICALLY for
    /// every budget from 4 ms to 12 ms — the giveaway that the collapse
    /// happened during acquisition and had nothing to do with the budget.
    ///
    /// So the ladder only climbs while there is a lock to keep fed. Misses are
    /// still COUNTED during search, because they are real and hiding them
    /// would make the report flattering; what changes is that they do not
    /// provoke a response that cannot help.
    void observe(double cost_us, bool may_shed = true) noexcept {
        if (!enabled_) return;

        // COUNTED BEFORE THE LEVEL MOVES, and the order is the whole point.
        //
        // decision() is read BEFORE a frame is processed and observe() is
        // called AFTER, so the frame whose cost is arriving now ran under the
        // level as it stands at this instant — not under the level this call
        // is about to set. Incrementing after the update counted "frames after
        // which the system was shedding", which is a different quantity and
        // wrong by one in both directions: it charged the stalling frame with
        // being degraded, when in fact that frame ran at FULL quality and that
        // is precisely why it was slow, and it let the last degraded frame go
        // uncounted because the ladder dropped to 0 on the same call.
        if (level_ > 0) ++shed_frames_;

        last_missed_ = cost_us > budget_us_;
        if (last_missed_) {
            ++misses_;
            const double over = cost_us - budget_us_;
            if (over > worst_overrun_us_) worst_overrun_us_ = over;
        }

        if (!may_shed) {
            // Nothing to protect. Stand the ladder down rather than holding a
            // rung that will be applied to the first frame after the lock
            // arrives — which is the worst possible frame to degrade.
            level_ = 0;
            comfortable_run_ = 0;
            return;
        }

        if (last_missed_) {
            // One frame is enough to climb. See kRecoverFrames for why the two
            // directions are not symmetric.
            if (level_ < kMaxLevel) ++level_;
            comfortable_run_ = 0;
        } else if (cost_us <= budget_us_ * kRecoverFraction) {
            if (++comfortable_run_ >= kRecoverFrames) {
                if (level_ > 0) --level_;
                comfortable_run_ = 0;
            }
        } else {
            // Inside budget but not comfortably. Hold the current rung: this
            // is the band where un-shedding would put the next frame back over
            // the line, and it is where a system under steady load should sit.
            comfortable_run_ = 0;
        }
    }

    // -----------------------------------------------------------------------
    /// What to shed on the frame about to be processed.
    [[nodiscard]] ShedDecision decision() const noexcept {
        ShedDecision d;
        if (!enabled_) return d;
        d.level       = level_;
        d.skip_median = level_ >= 1;
        d.shrink_roi  = level_ >= 2;
        return d;
    }

    [[nodiscard]] bool    enabled()          const noexcept { return enabled_; }
    [[nodiscard]] double  budget_us()        const noexcept { return budget_us_; }
    /// True if the frame most recently observed overran. Per-frame, for the
    /// record; `misses()` is the run total.
    [[nodiscard]] bool    last_missed()      const noexcept { return last_missed_; }
    [[nodiscard]] int64_t misses()           const noexcept { return misses_; }
    /// Frames processed in a shed configuration. NOT the same as misses():
    /// one miss sheds for at least kRecoverFrames afterwards, which is the
    /// number that says how much of the run ran degraded.
    [[nodiscard]] int64_t shed_frames()      const noexcept { return shed_frames_; }
    [[nodiscard]] int     level()            const noexcept { return level_; }
    [[nodiscard]] double  worst_overrun_us() const noexcept { return worst_overrun_us_; }

private:
    double  budget_us_        = 0.0;
    bool    enabled_          = false;
    int     level_            = 0;
    int     comfortable_run_  = 0;
    int64_t misses_           = 0;
    int64_t shed_frames_      = 0;
    bool    last_missed_      = false;
    double  worst_overrun_us_ = 0.0;
};

}  // namespace sat
