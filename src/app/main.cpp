// src/app/main.cpp — entry point for sat-tracker.
//
// Design §13.4 lists the full command line this must eventually grow into
// (--scenario, --video, --headless, --sweep, --no-ai, ...). At this checkpoint
// the binary handles the two arguments that CI and the smoke test need, and
// prints the derived-constants banner that design §1.4 asks to be logged at
// startup. Real CLI parsing lands with the scenario loader in Stage 3.

#include "sat/version.hpp"

#include "core/frames.hpp"
#include "core/units.hpp"

#include <cstdio>
#include <cstring>

namespace {

// ---------------------------------------------------------------------------
// The derived-constants banner.
//
// Design §1.4: "compute once, log at startup". These numbers are the ones the
// whole design argument rests on -- in particular that 20 px/frame of jitter is
// 75% of a 5 deg/s motor's authority, which is why the system must predict
// rather than react (§1.3). Printing them means a reviewer can check the claim
// against the running build rather than against the document.
// ---------------------------------------------------------------------------
void print_derived_constants() {
    // Spec defaults: rows 1, 3, 4, 5, 13, 23.
    const auto cam = sat::CameraGeometry::make(640, 480, 4.0, 3.0);
    const auto scr = sat::ScreenGeometry::make(2000, 2000, cam);

    const double ifov       = cam.ifov_urad();
    const double camera_hz  = 30.0;
    const double max_dps    = 5.0;
    const double jitter_pxf = 20.0;

    const double screen_deg   = sat::urad_to_deg(scr.extent_urad().x);
    const double fov_fraction = static_cast<double>(cam.pixel_count())
                              / static_cast<double>(scr.width * scr.height);
    const double pan_px_s     = sat::deg_to_urad(max_dps) / ifov;
    const double jitter_urad_s = sat::px_per_frame_to_urad_s(jitter_pxf, ifov, camera_hz);

    std::printf("derived constants (design §1.4):\n");
    std::printf("  IFOV                       = %.5f urad/px (%.6f deg/px)\n",
                ifov, sat::urad_to_deg(ifov));
    std::printf("  screen angular extent      = %.2f deg x %.2f deg\n", screen_deg, screen_deg);
    std::printf("  FOV fraction of screen     = %.2f%%\n", fov_fraction * 100.0);
    std::printf("  max pan @ %.1f deg/s         = %.1f px/s = %.1f px/frame @ %.0f Hz\n",
                max_dps, pan_px_s, pan_px_s / camera_hz, camera_hz);
    std::printf("  jitter %.0f px/frame          = %.0f urad/s = %.2f deg/s (%.0f%% of authority)\n",
                jitter_pxf, jitter_urad_s, sat::urad_to_deg(jitter_urad_s),
                100.0 * sat::urad_to_deg(jitter_urad_s) / max_dps);
    std::printf("  tracking error budget 10px = %.2f mrad\n", 10.0 * ifov / 1000.0);
}

void print_usage() {
    std::printf("usage: sat-tracker [options]\n\n");
    std::printf("  --version     print the version and build hash, then exit\n");
    std::printf("  --help        print this message\n");
    std::printf("\nThe full command line from design §13.4 (--scenario, --video,\n");
    std::printf("--headless, --sweep, --no-ai) arrives with the scenario loader.\n");
}

}  // namespace

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--version") == 0) {
            std::printf("%s %s+%s\n", SAT_PRODUCT_NAME, SAT_VERSION, SAT_GIT_HASH);
            return 0;
        }
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage();
            return 0;
        }
        std::fprintf(stderr, "sat-tracker: unrecognised option '%s'\n", argv[i]);
        print_usage();
        return 2;
    }

    std::printf("%s %s+%s\n\n", SAT_PRODUCT_NAME, SAT_VERSION, SAT_GIT_HASH);
    print_derived_constants();
    return 0;
}
