// core/profile.hpp — per-stage timing.
//
// Design §13.1: "processing_ms : per stage, p50 / p95 / p99 -- NEVER the mean",
// and decision 17 explains why: "A 0.3 ms mean hiding a 25 ms p99 is a broken
// control loop." CP 14.4 requires these percentiles to come out of the SHIPPED
// binary, not a special profiling build.
//
// ---------------------------------------------------------------------------
// THIS IS THE ONE PLACE A WALL CLOCK IS ALLOWED
// ---------------------------------------------------------------------------
// INV-3 forbids std::chrono in the simulation path. Timing is not in the
// simulation path: no measured duration is ever fed back into the simulation,
// and the recorded values are excluded from the reproducibility fingerprint
// (core/hash.hpp). Keeping the only chrono include in this file makes that
// boundary greppable -- CI can assert that `chrono` appears nowhere else under
// src/ except here and in the app layer.
//
// The histogram is fixed-size and lives in the collector, so recording a sample
// allocates nothing (INV-4).

#pragma once

#include <algorithm>
#include <array>
#include <chrono>   // see the note above -- the only sanctioned use
#include <cstdint>
#include <string_view>

namespace sat {

// ---------------------------------------------------------------------------
// wall — the sanctioned wall clock, for the two places outside this header
// that legitimately need one.
//
// INV-3's checker asserts that <chrono> appears nowhere under src/ except
// here and in the app and GUI layers, and the comment above says why: keeping
// the include in one file makes the boundary greppable. Two engine-side call
// sites nevertheless need real elapsed time —
//
//   * the P1-10 deadline governor, which has to know how long a frame ACTUALLY
//     took before it can say the frame missed its budget, and
//   * DecodeThread's stall timeout, which has to give up waiting on a
//     third-party decoder that has hung.
//
// Neither feeds a measured duration back into the simulation, and neither
// value enters the reproducibility fingerprint, so both are inside the
// invariant's intent. Routing them through this façade keeps them inside its
// LETTER too, which is better than widening the checker's exclusion list to
// cover two whole files — a blanket exclusion for engine/pipeline.cpp would
// retire the invariant over exactly the file it matters most in.
// ---------------------------------------------------------------------------
namespace wall {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/// Monotonic now. Never the system clock: a wall-clock step (NTP, DST) must
/// not be able to make a frame look like it took a negative amount of time.
[[nodiscard]] inline TimePoint now() noexcept { return Clock::now(); }

/// A time point `seconds` into the future, for condition_variable::wait_until.
[[nodiscard]] inline TimePoint deadline_in(double seconds) noexcept {
    return now() + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(seconds));
}

/// Microseconds elapsed since `since`. A default-constructed TimePoint means
/// "never started" and reads back as 0 rather than as the age of the epoch.
[[nodiscard]] inline double elapsed_us(TimePoint since) noexcept {
    if (since == TimePoint{}) return 0.0;
    return std::chrono::duration<double, std::micro>(now() - since).count();
}

}  // namespace wall


// ---------------------------------------------------------------------------
// Stage — every pipeline step we time separately.
//
// The list mirrors design §6.2 and §15 so the measured table can be compared
// line for line against the performance targets in the report.
// ---------------------------------------------------------------------------
enum class Stage : uint8_t {
    WorldAdvance = 0,
    Disturbance,
    GimbalStep,
    FrameAcquire,      ///< PARENT: render+degrade, or video crop. Not additive.
    BackgroundRender,  ///< the pedestal fill; §15's "Background render"
    EmitterSplat,      ///< §15's "Emitter splat, 8 substeps"
    DamageChain,       ///< §15's "Damage chain" — atmosphere, noise, defects
    Perception,        ///< PARENT: the whole detector, B6-B13. Not additive.
    Median,
    TopHat,
    SummedArea,
    MatchedFilter,
    Cfar,
    Grouping,
    Centroid,          ///< LEAF: §15's "Centroid + bias correction" only
    AiCandidate,
    AiCentroid,
    AiRecovery,
    Tracking,
    Supervisor,
    Control,
    Metrics,
    Snapshot,
    FrameTotal,        ///< the whole per-frame budget; compare against 0.85 ms
    kCount
};

[[nodiscard]] const char* stage_name(Stage s) noexcept;

// ---------------------------------------------------------------------------
// LatencyHistogram — log-spaced buckets over [1 us, 100 ms].
//
// A histogram rather than a reservoir of samples because a 120 s run at 30 Hz
// is 3600 frames x 20 stages, and we want percentiles from a sweep of 1500 runs
// without storing 100 million doubles. 128 log-spaced buckets give roughly 5%
// resolution on the quantile, which is far finer than the decisions we make
// from it.
// ---------------------------------------------------------------------------
class LatencyHistogram {
public:
    // -----------------------------------------------------------------------
    // RANGE AND RESOLUTION — and why the old ceiling was a reporting defect.
    //
    // The range used to end at 100 ms, and bucket_of() clamps anything at or
    // above the ceiling into the top bucket. quantile() then returns that
    // bucket's CENTRE, so every frame slower than ~91 ms reported as exactly
    // 95.6 ms — indistinguishable from a real measurement, and printed beside
    // genuine percentiles with nothing to mark it.
    //
    // That is not hypothetical. The BP-2 screen path's p99 read "95.602 ms" on
    // every run, to the last digit, across runs whose p50 varied by 30 %. It
    // was taken at face value in an external audit and turned into "10.5 FPS,
    // below spec row 20's 20 FPS minimum". The true figure was above 91 ms and
    // otherwise unknown — the instrument had saturated.
    //
    // 10 seconds, because the things that legitimately land in the tail are
    // startup frames: the first frame of a video run allocates the workspace,
    // fills the decoder's ring and JITs nothing, and on a loaded machine that
    // is hundreds of milliseconds. A ceiling has to be above the values it is
    // meant to measure, not above the ones it is meant to like.
    //
    // 256 buckets keeps the resolution at 6.5 % per bucket over the wider
    // range, slightly better than the 9.4 % it had before. Cost is 2 KB per
    // histogram, ~43 KB for the whole stage table, once.
    //
    // saturated() reports occupancy of the top bucket so a reader is TOLD when
    // a percentile is a floor rather than a value, instead of having to notice
    // that it never changes.
    // -----------------------------------------------------------------------
    static constexpr int    kBuckets = 256;
    static constexpr double kMinUs   = 1.0;            ///< 1 microsecond
    static constexpr double kMaxUs   = 10'000'000.0;   ///< 10 seconds

    void record(double microseconds) noexcept {
        ++count_;
        sum_us_ += microseconds;
        if (microseconds > max_us_) max_us_ = microseconds;
        ++buckets_[bucket_of(microseconds)];
    }

    void reset() noexcept {
        buckets_.fill(0);
        count_  = 0;
        sum_us_ = 0.0;
        max_us_ = 0.0;
    }

    /// Linear-interpolated quantile in microseconds. q in [0, 1].
    [[nodiscard]] double quantile(double q) const noexcept {
        if (count_ == 0) return 0.0;
        const uint64_t target = static_cast<uint64_t>(q * static_cast<double>(count_));
        uint64_t cum = 0;
        for (int i = 0; i < kBuckets; ++i) {
            cum += buckets_[i];
            if (cum > target) return bucket_centre(i);
        }
        return max_us_;
    }

    [[nodiscard]] double   p50()   const noexcept { return quantile(0.50); }
    [[nodiscard]] double   p95()   const noexcept { return quantile(0.95); }
    [[nodiscard]] double   p99()   const noexcept { return quantile(0.99); }
    [[nodiscard]] double   max()   const noexcept { return max_us_; }

    /// Samples that landed in the top bucket, i.e. at or above the ceiling.
    ///
    /// Non-zero means any percentile at or above the corresponding rank is a
    /// LOWER BOUND, not a measurement. max() is still exact — it is tracked
    /// outside the buckets — so a saturated histogram can still say how bad
    /// the worst frame was, just not where the 99th percentile sits.
    [[nodiscard]] uint64_t saturated() const noexcept { return buckets_[kBuckets - 1]; }
    [[nodiscard]] uint64_t count() const noexcept { return count_; }

    /// Reported alongside the percentiles, never instead of them. It is in the
    /// output only so a reader can see mean-versus-p99 divergence for themselves.
    [[nodiscard]] double mean() const noexcept {
        return count_ ? sum_us_ / static_cast<double>(count_) : 0.0;
    }

    /// Merge another histogram in. Used by --sweep to combine worker processes.
    void merge(const LatencyHistogram& o) noexcept {
        for (int i = 0; i < kBuckets; ++i) buckets_[i] += o.buckets_[i];
        count_  += o.count_;
        sum_us_ += o.sum_us_;
        max_us_  = std::max(max_us_, o.max_us_);
    }

private:
    // Bucket i covers [kMinUs * r^i, kMinUs * r^(i+1)) where r = (kMaxUs/kMinUs)^(1/kBuckets).
    // Implemented with a lookup-free log so it costs one log() per sample --
    // negligible next to the work being measured.
    [[nodiscard]] static int bucket_of(double us) noexcept;
    [[nodiscard]] static double bucket_centre(int i) noexcept;

    std::array<uint64_t, kBuckets> buckets_{};
    uint64_t count_  = 0;
    double   sum_us_ = 0.0;
    double   max_us_ = 0.0;
};

// ---------------------------------------------------------------------------
// StageTimers — one histogram per Stage, plus the scoped helper that fills them.
// ---------------------------------------------------------------------------
class StageTimers {
public:
    void record(Stage s, double microseconds) noexcept {
        h_[static_cast<size_t>(s)].record(microseconds);
    }
    [[nodiscard]] const LatencyHistogram& operator[](Stage s) const noexcept {
        return h_[static_cast<size_t>(s)];
    }
    [[nodiscard]] LatencyHistogram& operator[](Stage s) noexcept {
        return h_[static_cast<size_t>(s)];
    }
    void reset() noexcept { for (auto& x : h_) x.reset(); }
    void merge(const StageTimers& o) noexcept {
        for (size_t i = 0; i < h_.size(); ++i) h_[i].merge(o.h_[i]);
    }

private:
    std::array<LatencyHistogram, static_cast<size_t>(Stage::kCount)> h_{};
};

/// RAII timer. Reads the steady clock on construction and on destruction and
/// files the difference. Costs ~20 ns per scope, which is why it can stay on in
/// release builds.
class ScopedTimer {
public:
    ScopedTimer(StageTimers& t, Stage s) noexcept
        : timers_(&t), stage_(s), start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        const auto end = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(end - start_).count();
        timers_->record(stage_, us);
    }

    ScopedTimer(const ScopedTimer&)            = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    StageTimers*                                       timers_;
    Stage                                              stage_;
    std::chrono::time_point<std::chrono::steady_clock> start_;
};

/// The same, but tolerant of a null StageTimers*. Used where a component may
/// or may not have been attached to the engine's profiler — the synthetic
/// source is built directly by several tests and by the centroid harness, and
/// none of those want to construct a timer set just to render a frame.
class OptionalTimer {
public:
    OptionalTimer(StageTimers* t, Stage s) noexcept
        : timers_(t), stage_(s),
          start_(t ? std::chrono::steady_clock::now()
                   : std::chrono::time_point<std::chrono::steady_clock>{}) {}

    ~OptionalTimer() {
        if (!timers_) return;
        const auto end = std::chrono::steady_clock::now();
        timers_->record(stage_,
                        std::chrono::duration<double, std::micro>(end - start_).count());
    }

    OptionalTimer(const OptionalTimer&)            = delete;
    OptionalTimer& operator=(const OptionalTimer&) = delete;

private:
    StageTimers*                                       timers_;
    Stage                                              stage_;
    std::chrono::time_point<std::chrono::steady_clock> start_;
};

}  // namespace sat

// SAT_ZONE(timers, Stage::Median) — one line at the top of a stage function.
// The token pasting gives each zone a unique variable name so two zones can
// nest in the same scope.
#define SAT_ZONE_CAT_(a, b) a##b
#define SAT_ZONE_CAT(a, b)  SAT_ZONE_CAT_(a, b)
#define SAT_ZONE(timers, stage) \
    ::sat::ScopedTimer SAT_ZONE_CAT(sat_zone_, __LINE__)((timers), (stage))

// SAT_ZONE_OPT(timers_ptr, Stage::X) — the same, for code that may or may not
// have been handed a StageTimers. A null pointer means "not being profiled",
// which is the normal case in unit tests and in the accuracy harnesses.
#define SAT_ZONE_OPT(timers_ptr, stage)                         \
    ::sat::OptionalTimer SAT_ZONE_CAT(sat_zone_, __LINE__)((timers_ptr), (stage))
