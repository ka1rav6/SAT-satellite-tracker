#include "engine/snapshot.hpp"

namespace sat {

FrameFingerprint fingerprint(const SimSnapshot& s) noexcept {
    FrameFingerprint f{};
    f.frame = s.frame;

    // The image: the most sensitive single summary of everything upstream.
    f.image = fnv1a(std::span<const uint8_t>(s.preview));

    // Pointing, bit-exact. These are pure arithmetic on our own state, with no
    // libm in the path, so there is nothing to excuse.
    uint64_t h = kFnvOffsetBasis;
    h = hash_vec2(s.boresight_cmd,  h);
    h = hash_vec2(s.boresight_true, h);
    h = hash_vec2(s.cmd_rate,       h);
    h = hash_vec2(s.gimbal_rate,    h);
    f.boresight = h;

    // Detection, at a declared tolerance (design §11.4).
    h = kFnvOffsetBasis;
    h = hash_value(static_cast<uint8_t>(s.detected ? 1 : 0), h);
    h = hash_value(static_cast<int32_t>(s.candidate_count), h);
    if (s.detected) {
        h = hash_quantised(s.detection_img.x, 1e-6, h);
        h = hash_quantised(s.detection_img.y, 1e-6, h);
        h = hash_quantised(static_cast<double>(s.detection_snr), 1e-4, h);
    }
    f.detection = h;

    // Discrete decisions: integers, always exact.
    h = kFnvOffsetBasis;
    h = hash_value(static_cast<uint8_t>(s.mode), h);
    h = hash_value(static_cast<int32_t>(s.track_state), h);
    f.mode = h;

    // Filter state lands here at Stage 6.
    f.track = kFnvOffsetBasis;

    return f;
}

}  // namespace sat
