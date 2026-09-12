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
// Stage — every pipeline step we time separately.
//
// The list mirrors design §6.2 and §15 so the measured table can be compared
// line for line against the performance targets in the report.
// ---------------------------------------------------------------------------
enum class Stage : uint8_t {
    WorldAdvance = 0,
    Disturbance,
    GimbalStep,
    FrameAcquire,      ///< render+degrade, or video crop
    Median,
    TopHat,
    SummedArea,
    MatchedFilter,
    Cfar,
    Grouping,
    Centroid,
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
    static constexpr int    kBuckets = 128;
    static constexpr double kMinUs   = 1.0;        ///< 1 microsecond
    static constexpr double kMaxUs   = 100'000.0;  ///< 100 milliseconds

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

}  // namespace sat

// SAT_ZONE(timers, Stage::Median) — one line at the top of a stage function.
// The token pasting gives each zone a unique variable name so two zones can
// nest in the same scope.
#define SAT_ZONE_CAT_(a, b) a##b
#define SAT_ZONE_CAT(a, b)  SAT_ZONE_CAT_(a, b)
#define SAT_ZONE(timers, stage) \
    ::sat::ScopedTimer SAT_ZONE_CAT(sat_zone_, __LINE__)((timers), (stage))
