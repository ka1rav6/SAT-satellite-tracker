// metrics/centroid_log.cpp

#include "metrics/centroid_log.hpp"


namespace sat {

std::string CentroidLog::header_text(const CentroidLogHeader& h) {
    char buf[1024];
    std::string out = "# SAT centroid log v1\n";

    std::snprintf(buf, sizeof buf, "# source=%s  mode=%s  build=%s  utc=%s\n",
                  h.source.c_str(), h.mode.c_str(), h.build.c_str(), h.utc.c_str());
    out += buf;

    // Geometry and rates. Everything needed to convert either coordinate pair
    // into the other, or into angles, without consulting anything else —
    // which is §13.2's "the header alone must be sufficient".
    std::snprintf(buf, sizeof buf,
                  "# screen_px=%dx%d  camera_px=%dx%d  fps=%.3f  ifov_urad=%.2f\n",
                  h.screen_w, h.screen_h, h.camera_w, h.camera_h, h.fps, h.ifov_urad);
    out += buf;

    std::snprintf(buf, sizeof buf, "# onnxruntime=%s  ai_enabled=%s\n",
                  h.onnxruntime.c_str(), h.ai_enabled ? "true" : "false");
    out += buf;

    out += "# columns: ";
    out += kColumns;
    out += "\n";
    return out;
}

bool CentroidLog::open(const std::string& path, const CentroidLogHeader& h) {
    close();
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_) return false;

    const std::string head = header_text(h);
    std::fwrite(head.data(), 1, head.size(), f_);

    // FLUSHED IMMEDIATELY. §13.2: "Write the header at file open (A9), not at
    // close — so an aborted run still leaves a valid file." Buffering it would
    // defeat the whole point, because the abort cases that matter (a sweep
    // worker hitting its timeout, a kill during a long run) never flush.
    std::fflush(f_);
    rows_ = 0;
    return true;
}

void CentroidLog::write(const FrameRecord& r, const ScreenGeometry& screen) {
    if (!f_) return;

    // §13.2's bore_x/bore_y are in SCREEN pixels, the same frame as
    // cx_screen/cy_screen — so a reader can subtract the two pairs and get the
    // offset directly. The COMMANDED boresight, not the true one: the true one
    // is simulator state the system does not know, and putting it in a
    // submitted artifact would be claiming knowledge we do not have.
    const Pixel2 bore = screen.to_pixel(r.boresight_cmd);

    // ----------------------------------------------------------------------
    // INV-9, mechanised. When there is no detection the seven measurement
    // columns are EMPTY. Not zero, not the previous row's value, not an
    // interpolation — empty, so that any reader distinguishes "we did not see
    // it" from "we saw it at the origin".
    //
    // Note there is no `prev_` member in this class. That is deliberate: a
    // stale value cannot be written because no stale value is kept.
    // ----------------------------------------------------------------------
    if (r.detected) {
        std::fprintf(f_,
                     "%lld,%.4f,%s,%.3f,%.3f,%.3f,%.3f,%.4f,%.1f,%u,%u,%.3f,%.3f\n",
                     static_cast<long long>(r.frame), r.time_s,
                     track_mode_name(r.mode),
                     r.detection_screen.x, r.detection_screen.y,
                     r.detection_img.x,    r.detection_img.y,
                     static_cast<double>(r.centroid_sigma_px),
                     static_cast<double>(r.detection_snr),
                     static_cast<unsigned>(r.detection_area_px),
                     static_cast<unsigned>(r.detection_size_est_px),
                     bore.x, bore.y);
    } else {
        std::fprintf(f_, "%lld,%.4f,%s,,,,,,,,,%.3f,%.3f\n",
                     static_cast<long long>(r.frame), r.time_s,
                     track_mode_name(r.mode), bore.x, bore.y);
    }
    ++rows_;
}

void CentroidLog::close() {
    if (f_) {
        std::fclose(f_);
        f_ = nullptr;
    }
}

}  // namespace sat
