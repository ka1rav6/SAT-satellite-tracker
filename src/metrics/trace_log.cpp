// metrics/trace_log.cpp

#include "metrics/trace_log.hpp"

#include <cstdio>

namespace sat {

std::string TraceLog::header_text(const TraceLogHeader& h) {
    char buf[512];
    std::string out = "# SAT control trace v1 — DIAGNOSTIC, CONTAINS TRUTH, NOT GRADED\n";

    std::snprintf(buf, sizeof buf, "# source=%s  build=%s  utc=%s  ifov_urad=%.4f\n",
                  h.source.c_str(), h.build.c_str(), h.utc.c_str(), h.ifov_urad);
    out += buf;

    // The gains are in the header because a trace is only interpretable
    // alongside the configuration that produced it, and CP 10.1's whole point
    // is comparing two traces that differ in exactly one of these numbers.
    std::snprintf(buf, sizeof buf, "# kp=%.4f  ki=%.4f  kd=%.4f  k_ff=%.4f\n",
                  h.kp, h.ki, h.kd, h.k_ff);
    out += buf;

    out += "frame,t,mode,err_px,err_x_px,err_y_px,"
           "cmd_rate_x,cmd_rate_y,gimbal_rate_x,gimbal_rate_y,"
           "integ_x,integ_y,est_rate_x,est_rate_y,saturated,"
           "imm_cv,imm_ca,imm_ct,imm_turn_rate\n";
    return out;
}

bool TraceLog::open(const std::string& path, const TraceLogHeader& h) {
    close();
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_) return false;
    const std::string head = header_text(h);
    std::fwrite(head.data(), 1, head.size(), f_);
    std::fflush(f_);                 // see the note in centroid_log.cpp
    rows_ = 0;
    return true;
}

void TraceLog::write(const TraceSample& s) {
    if (!f_) return;
    // INV-9's principle applied to a different column: when there is no truth
    // this frame, the error columns are EMPTY rather than zero. A zero would
    // read as "perfectly pointed" and would drag any mean computed over the
    // file towards zero, which is the exact failure mode of writing a
    // placeholder instead of a gap.
    if (s.truth_valid) {
        std::fprintf(f_, "%lld,%.4f,%s,%.4f,%.4f,%.4f,",
                     static_cast<long long>(s.frame), s.time_s, s.mode,
                     s.tracking_error_px, s.err_x_px, s.err_y_px);
    } else {
        std::fprintf(f_, "%lld,%.4f,%s,,,,",
                     static_cast<long long>(s.frame), s.time_s, s.mode);
    }
    std::fprintf(f_, "%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%d,"
                     "%.4f,%.4f,%.4f,%.5f\n",
                 s.cmd_rate_x, s.cmd_rate_y, s.gimbal_rate_x, s.gimbal_rate_y,
                 s.integ_x, s.integ_y, s.est_rate_x, s.est_rate_y,
                 s.saturated ? 1 : 0,
                 s.imm_cv, s.imm_ca, s.imm_ct, s.imm_turn_rate);
    ++rows_;
}

void TraceLog::close() {
    if (f_) { std::fclose(f_); f_ = nullptr; }
}

}  // namespace sat
