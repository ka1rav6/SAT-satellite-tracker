// metrics/trace_log.hpp — the per-frame CONTROL trace.
//
// ---------------------------------------------------------------------------
// WHY THIS IS NOT centroid.csv, AND MUST NOT BECOME IT
// ---------------------------------------------------------------------------
// Design §13.2 fixes the format of centroid.csv, and that file is a GRADED
// deliverable: it is what the evaluator reads, and every column in it is
// something the system genuinely knows. Truth is deliberately absent from it,
// because a graded artifact that quietly contains the answer key is worthless
// as evidence.
//
// Stage 10 needs the opposite file. Tuning a control loop means looking at the
// error against truth, frame by frame — the pointing error, the commanded
// rate, the integrator, whether the plant was against its stops. Those are
// SIMULATOR-ONLY quantities. Putting them in centroid.csv would break §13.2's
// format and put truth into a graded file; putting them nowhere means every
// control checkpoint from 10.1 to 10.6 is argued from a single summary number.
//
// So there are two files with two different jobs, and this one says so in its
// own header line: it is a diagnostic, it contains truth, it is never graded,
// and it is written only when explicitly asked for.
//
// ---------------------------------------------------------------------------
// WHAT USES IT
// ---------------------------------------------------------------------------
//   CP 10.1  the feedforward on/off comparison plot
//   CP 10.2  the slew settling trace, which is how "no ringing" is checked
//   CP 10.3  the residual after platform-drift cancellation
//   CP 10.6  error against disturbance level, and the saturation fraction
//
// It is off by default and costs one fprintf per frame when on, which is why
// the speed measurement (--bench) never enables it.

#pragma once

#include "core/frames.hpp"

#include <cstdio>
#include <string>

namespace sat {

struct FrameRecord;

/// One line of provenance, so a trace file found on its own is interpretable.
struct TraceLogHeader {
    std::string source;      ///< scenario path
    std::string build;       ///< git hash
    std::string utc;         ///< when
    double      ifov_urad = 1.0;
    double      kp = 0.0, ki = 0.0, kd = 0.0, k_ff = 0.0;
};

/// One frame's worth of control state. Assembled by the caller, because the
/// pieces live in three different objects (the record, the gimbal, the
/// controller) and metrics/ must not reach into engine/ to fetch them.
struct TraceSample {
    int64_t frame = 0;
    double  time_s = 0.0;
    const char* mode = "";
    bool    truth_valid = false;
    double  tracking_error_px = 0.0;
    double  err_x_px = 0.0, err_y_px = 0.0;   ///< signed, so lag vs lead is visible
    double  cmd_rate_x = 0.0, cmd_rate_y = 0.0;
    double  gimbal_rate_x = 0.0, gimbal_rate_y = 0.0;
    double  integ_x = 0.0, integ_y = 0.0;
    double  est_rate_x = 0.0, est_rate_y = 0.0;
    bool    saturated = false;
};

// ---------------------------------------------------------------------------
// TraceLog — an append-only CSV.
//
// Same ownership shape as CentroidLog: opens a FILE*, writes the header
// immediately, closes in the destructor. The header is flushed at open for the
// reason §13.2 gives — a run killed by a sweep timeout must still leave a file
// that says what it was.
// ---------------------------------------------------------------------------
class TraceLog {
public:
    TraceLog() = default;
    ~TraceLog() { close(); }
    TraceLog(const TraceLog&) = delete;
    TraceLog& operator=(const TraceLog&) = delete;

    [[nodiscard]] bool open(const std::string& path, const TraceLogHeader& h);
    void write(const TraceSample& s);
    void close();

    [[nodiscard]] bool is_open() const noexcept { return f_ != nullptr; }
    [[nodiscard]] int64_t rows() const noexcept { return rows_; }

    /// Exposed so a test can assert the header without touching a filesystem.
    [[nodiscard]] static std::string header_text(const TraceLogHeader& h);

private:
    std::FILE* f_ = nullptr;
    int64_t    rows_ = 0;
};

}  // namespace sat
