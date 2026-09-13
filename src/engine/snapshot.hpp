// engine/snapshot.hpp — CP 2.3 and CP 2.5.
//
// Two jobs, deliberately in one type:
//
//   1. SimSnapshot is what the simulation publishes to the display thread every
//      frame through the triple buffer (design §6.2 step B30/B31). The GUI is
//      deferred to CP 15.0 by the §14.0 amendment, but the SEAM is built now —
//      that is the whole justification for deferring, and a seam that is not
//      exercised is not a seam.
//
//   2. It is also what gets hashed for the reproducibility fingerprint. That is
//      not a coincidence: the snapshot is, by construction, everything about a
//      frame that is externally observable. If two runs agree on every snapshot
//      they agree on everything anyone can see.
//
// ---------------------------------------------------------------------------
// WHY THE PREVIEW IMAGE IS PRE-SIZED
// ---------------------------------------------------------------------------
// The snapshot owns a copy of the frame, because the display thread reads it
// asynchronously and the source's own buffer will have been overwritten by then.
// That copy must not allocate (INV-4), so the vector is sized once when the
// triple buffer is constructed from a prototype and only ever assigned into
// afterwards.

#pragma once

#include "core/hash.hpp"
#include "core/units.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

/// Mode FSM state (design §10.4). Defined here rather than in control/ because
/// the snapshot and the centroid log both need to name it, and neither may
/// depend on the controller.
enum class TrackMode : uint8_t {
    Idle = 0, Search, Detect, Acquire, Track, Reacquire, Handover, Safe
};

[[nodiscard]] const char* track_mode_name(TrackMode m) noexcept;

// ---------------------------------------------------------------------------
// SimSnapshot
// ---------------------------------------------------------------------------
struct SimSnapshot {
    int64_t frame  = 0;
    double  time_s = 0.0;

    // --- pointing ----------------------------------------------------------
    Angle2 boresight_cmd{};    ///< what the controller asked for
    Angle2 boresight_true{};   ///< where the camera actually pointed
    Rate2  cmd_rate{};
    Rate2  gimbal_rate{};

    // --- perception --------------------------------------------------------
    bool   detected = false;
    Pixel2 detection_img{};
    Pixel2 detection_screen{};
    float  detection_snr = 0.0f;
    float  centroid_sigma_px = 0.0f;
    int    candidate_count = 0;

    // --- tracking ----------------------------------------------------------
    TrackMode mode = TrackMode::Idle;
    int       track_state = 0;      ///< lifecycle enum, Stage 6

    // --- truth and metrics (display and metrics only; INV-1) ---------------
    bool   truth_valid = false;
    Pixel2 truth_screen{};
    bool   truth_in_fov = false;
    double centroid_error_px = 0.0;
    double tracking_error_px = 0.0;

    // --- the image the detector saw ---------------------------------------
    std::vector<uint8_t> preview;
    int preview_w = 0, preview_h = 0;

    /// Size the preview buffer. Call once, on the prototype handed to the
    /// triple buffer's constructor, so that publishing never allocates.
    void reserve_preview(int w, int h) {
        preview_w = w;
        preview_h = h;
        preview.assign(static_cast<size_t>(w) * static_cast<size_t>(h), 0);
    }

    /// Copy a frame in. Silently ignores a mismatched size rather than
    /// reallocating, because reallocating inside a frame would trip the INV-4
    /// allocation trap — and a size mismatch means the prototype was built
    /// wrong, which is a startup bug, not a per-frame condition.
    void set_preview(std::span<const uint8_t> src) {
        if (src.size() != preview.size()) return;
        std::copy(src.begin(), src.end(), preview.begin());
    }
};

// ---------------------------------------------------------------------------
// fingerprint — CP 2.5's snapshot_hash().
//
// "FNV-1a over frame, boresight, detection, mode."
//
// What goes in, and what deliberately does not:
//
//   IN   the 8-bit image, because it is the single most sensitive summary of
//        the world, the camera and the whole degradation chain. A divergence
//        anywhere upstream shows up here.
//   IN   both boresights, at full double precision.
//   IN   the detection, quantised to a micro-pixel. Design §11.4 asks for
//        exactly this distinction: values that pass through libm (or, later, a
//        neural network) are hashed at a declared tolerance rather than claimed
//        bit-exact. A micro-pixel is four orders of magnitude below the graded
//        metric.
//   IN   mode and track state, as integers — always exact, which is why §11.4
//        recommends hashing "the discrete decisions, not raw model outputs".
//
//   OUT  anything measured with a clock. Timings vary with machine load by
//        design and including them would make the fingerprint meaningless.
//   OUT  the truth fields. They are an input to the simulation, not an output
//        of the algorithm, and including them would mask a divergence in the
//        tracker behind agreement in the world.
// ---------------------------------------------------------------------------
[[nodiscard]] FrameFingerprint fingerprint(const SimSnapshot& s) noexcept;

}  // namespace sat
