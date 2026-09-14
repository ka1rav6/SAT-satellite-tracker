#include "perception/pipeline.hpp"

#include "perception/matched.hpp"
#include "perception/median.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

bool PerceptionWorkspace::allocate(Arena& arena, int width, int height, int se_size) {
    const size_t n    = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t satn = SummedArea::elements(width, height);

    filtered  = arena.alloc<uint8_t>(n);
    tophat    = arena.alloc<int16_t>(n);
    sat_sum   = arena.alloc<int64_t>(satn);
    sat_sumsq = arena.alloc<uint64_t>(satn);
    response   = arena.alloc<float>(n);
    response_q = arena.alloc<int16_t>(n);
    resp_sum   = arena.alloc<int64_t>(satn);
    resp_sumsq = arena.alloc<uint64_t>(satn);
    scale      = arena.alloc<uint8_t>(n);
    mask      = arena.alloc<uint8_t>(n);
    snr       = arena.alloc<float>(n);

    morph.a       = arena.alloc<uint8_t>(n);
    morph.b       = arena.alloc<uint8_t>(n);
    morph.c       = arena.alloc<uint8_t>(n);
    morph.scratch = arena.alloc<uint8_t>(
        MorphWorkspace::scratch_bytes(width, height, se_size));

    // Worst case for runs: alternating set/clear gives one run per two columns
    // per row. CFAR's mask is far sparser, but sizing for the worst case means
    // a pathological frame degrades gracefully instead of overrunning.
    const size_t max_runs = static_cast<size_t>(width / 2 + 1) * static_cast<size_t>(height);
    grouping.runs   = arena.alloc<Run>(max_runs);
    grouping.parent = arena.alloc<int32_t>(max_runs);
    grouping.rank   = arena.alloc<int32_t>(max_runs);

    return !filtered.empty() && !tophat.empty() && !sat_sum.empty() &&
           !sat_sumsq.empty() && !response.empty() && !response_q.empty() &&
           !resp_sum.empty() && !resp_sumsq.empty() && !scale.empty() &&
           !mask.empty() && !snr.empty() && !morph.a.empty() &&
           !grouping.runs.empty();
}

bool passes_gate(const BlobAccum& b, const PerceptionParams& p) noexcept {
    const int area = static_cast<int>(b.n);
    if (area < p.min_area || area > p.max_area) return false;
    if (b.fill_ratio() < p.min_fill) return false;
    if (b.aspect() > p.max_aspect) return false;
    return true;
}

float centroid_sigma(float snr, int size_est_px) noexcept {
    // Design §10.1.4. The theoretical bound from §10.1.1 is sigma >= w/(2*SNR);
    // this is that bound used directly as the estimate.
    //
    // The 0.03 px floor is not arbitrary: below it the estimate would claim
    // accuracy better than the renderer's own ground truth is measured to, and
    // an over-confident sigma fed into the Kalman R (§10.2) makes the filter
    // trust a measurement more than it deserves — which is how a tracker locks
    // onto a decoy and refuses to let go.
    return std::max(0.03f, static_cast<float>(size_est_px) / (2.0f * std::max(snr, 1.0f)));
}

void ClassicalPerception::process(std::span<const uint8_t> pixels,
                                  int width, int height,
                                  const PerceptionWorkspace& ws,
                                  std::vector<Detection>& out) {
    out.clear();
    last_blobs_ = 0;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || pixels.size() < n) return;

    // --- B6: median 3x3 ---------------------------------------------------
    std::span<const uint8_t> src = pixels;
    if (params_.median_enabled && ws.filtered.size() >= n) {
        median_3x3(pixels, ws.filtered, width, height);
        src = ws.filtered;
    }

    // --- B7: top-hat background removal -----------------------------------
    const int se = structuring_element_size(params_.target_size_px);
    top_hat(src, ws.tophat, width, height, se, ws.morph);

    // --- B8: summed-area tables over the top-hat --------------------------
    SummedArea sa{ws.sat_sum, ws.sat_sumsq, width, height};
    build_sat(ws.tophat, width, height, sa);

    // --- B9: multi-scale matched filter -----------------------------------
    //
    // Two products from this stage, and they are used for different things:
    //   ws.scale     the winning scale per pixel — a free size estimate
    //                (§9.4.4), feeding the centroid window and §10.1.4's sigma.
    //   ws.response  the response at ONE scale, the one nearest the configured
    //                target size, used for the detection threshold below.
    //
    // Detection deliberately does NOT use the max-over-scales response. A
    // maximum over six correlated scales is an order statistic: its background
    // has a higher mean and a markedly higher variance than any single scale's,
    // so CFAR raises its threshold to match and misses the target. Measured on
    // CP 5.9's worst case, thresholding the max missed the beacon entirely in
    // 23 of 120 frames; a single scale missed none.
    matched_filter(sa, width, height, ws.response, ws.scale);
    matched_filter_at_scale(sa, width, height,
                            nearest_scale(params_.target_size_px), ws.response);

    // --- B10: CFAR, on the MATCHED-FILTER RESPONSE -------------------------
    //
    // Design §6.2 runs B9 (matched filter) then B10 (CFAR threshold), and the
    // order is the point: the matched filter integrates the beacon over its
    // whole area, so a 10x10 target gains a factor of ~sqrt(100) = 10 in SNR
    // before any threshold is applied. Thresholding the raw top-hat throws that
    // away and asks each pixel to clear the bar alone.
    //
    // An earlier version did exactly that, with a plausible-sounding
    // justification: that thresholding a box-summed response against statistics
    // gathered from boxes would double-count the smoothing. The concern is real
    // but the conclusion was wrong — the fix is to gather the statistics from
    // the RESPONSE's own distribution, which is what the second summed-area
    // table below is for, not to skip the matched filter.
    //
    // It cost 9% of frames at CP 5.9's worst case. In fog the beacon's contrast
    // is 42 grey levels against read noise of sigma 20, so per-pixel SNR is
    // about 2.1 and CFAR at k = 3.9 fired on only the brightest few pixels. The
    // blob came out as a 1-10 pixel fragment and the shape gate's 12 px area
    // floor then — correctly — rejected it as noise. The detector was throwing
    // away real targets because it was asking single pixels to do a job the
    // matched filter exists to do.
    //
    // Quantised to int16 so the summed-area table stays integer, for the
    // precision and reproducibility reasons in §9.4.3. The response is bounded
    // by max_tophat * k_max / sqrt(k_max) = 255 * 20 = 5100, comfortably inside
    // int16.
    for (size_t i = 0; i < n; ++i) {
        ws.response_q[i] = static_cast<int16_t>(
            std::clamp(static_cast<int>(std::lround(ws.response[i])), -32768, 32767));
    }
    SummedArea resp_sa{ws.resp_sum, ws.resp_sumsq, width, height};
    build_sat(ws.response_q, width, height, resp_sa);
    cfar_mask(resp_sa, width, height, params_.cfar, ws.mask, ws.snr);

    // ...and UNION with CFAR on the top-hat itself.
    //
    // The two detectors fail in opposite directions, and measurement on CP
    // 5.9's worst case showed both failures clearly:
    //
    //   top-hat alone   the beacon's bright core fires, its dim rim does not,
    //                   so the blob comes out as a 1-10 px fragment that the
    //                   shape gate then rejects on area. 7 of 120 frames.
    //   response alone  solid blobs (mean area 212 vs 45), but the response is
    //                   a MAX over six correlated scales, and an order
    //                   statistic inflates both the mean and the variance of
    //                   the background. That raises CFAR's threshold and the
    //                   beacon is missed outright. 23 of 120 frames.
    //
    // Neither is a tuning problem; they are different detectors with different
    // blind spots. Taking the union gives the response's sensitivity to
    // extended low-contrast targets AND the top-hat's sensitivity to sharp
    // ones, and the grouping pass merges whatever fires into a single blob.
    //
    // The cost is one extra CFAR pass. The cost of not doing it was 9% of
    // frames one way and 19% the other, against a checkpoint that asks for 95%.
    //
    // The false-alarm rate of a union is at most the sum, so the stated Pfa
    // becomes 2*Q(k) ~ 9.6e-5 rather than 4.8e-5 — still a number that can be
    // predicted and defended, which is the property §9.4.5 actually cares about.
    for (size_t i = 0; i < n; ++i) {
        const CfarResult t = cfar_at(sa, static_cast<int>(i % static_cast<size_t>(width)),
                                     static_cast<int>(i / static_cast<size_t>(width)),
                                     params_.cfar);
        if (t.detected) {
            ws.mask[i] = 1u;
            // Keep the stronger of the two SNRs, so candidate ranking reflects
            // whichever detector saw the target more clearly.
            ws.snr[i] = std::max(ws.snr[i], t.snr);
        }
    }

    // --- B11: grouping -----------------------------------------------------
    //
    // Moments are weighted by the TOP-HAT, not by the response. The mask says
    // WHERE the target is; the centroid must be computed from the sharpest
    // available image of it, and the matched filter has deliberately blurred
    // the response over a 5-20 px box. Using the response as the weight would
    // pull every centroid toward the centre of its own smoothing kernel.
    last_blobs_ = group_components(ws.mask, ws.tophat, width, height,
                                   ws.grouping, blobs_);

    // --- B12/B13: gate, then centroid each survivor -----------------------
    for (const BlobAccum& b : blobs_) {
        if (!passes_gate(b, params_)) continue;

        Detection d{};
        // -----------------------------------------------------------------
        // B13: centroid. §10.1.2's estimator, then §10.1.3's bias correction.
        //
        // The blob's centre of mass seeds the window rather than the brightest
        // pixel. For spec row 9's default square beacon the top-hat is flat
        // across the shape, so argmax is degenerate and lands on whichever
        // corner noise favours — a window centred there cuts off most of the
        // beacon. The harness measured 1.4 px of error from exactly that, with
        // the giveaway that it was UNCORRELATED between noise realisations at
        // the same sub-pixel phase.
        // -----------------------------------------------------------------
        const Pixel2 seed = b.centroid();
        const int win = std::max(2, params_.target_size_px / 2 + 1);
        d.centroid_image = centroid_estimate(params_.centroid_kind, b,
                                             ws.tophat, ws.response,
                                             width, height, seed, win);
        d.area_px    = static_cast<uint16_t>(std::min<int64_t>(b.n, 65535));
        d.bbox_w     = static_cast<uint16_t>(b.width());
        d.bbox_h     = static_cast<uint16_t>(b.height());
        d.fill_ratio = b.fill_ratio();
        d.aspect     = b.aspect();
        d.peak       = b.peak;
        d.integrated = static_cast<float>(b.sw);

        // Sample the per-pixel maps at the centroid, clamped into the image.
        const int cx = std::clamp(static_cast<int>(std::lround(d.centroid_image.x)),
                                  0, width - 1);
        const int cy = std::clamp(static_cast<int>(std::lround(d.centroid_image.y)),
                                  0, height - 1);
        const size_t idx = static_cast<size_t>(cy) * static_cast<size_t>(width)
                         + static_cast<size_t>(cx);
        d.snr         = ws.snr[idx];
        d.size_est_px = ws.scale[idx];
        d.centroid_sigma_est = centroid_sigma(d.snr, d.size_est_px);

        // The correction is applied AFTER size_est_px and snr are known,
        // because the table is indexed by both. An unmeasured cell leaves the
        // estimate alone (perception/centroid/bias.hpp), so a build whose
        // table has not been calibrated behaves exactly as it did before
        // Stage 9 rather than shifting by an invented amount.
        if (params_.correct_centroid_bias) {
            d.centroid_image = BiasTable::builtin().correct(
                params_.centroid_kind, d.size_est_px, d.snr, d.centroid_image);
        }

        out.push_back(d);
    }

    // Strongest first, so a caller that takes the top N takes the best N.
    //
    // stable_sort, not sort. Design §2 INV-3: "Never std::sort on a key that can
    // tie — use std::stable_sort or add an index tiebreaker." SNR ties are
    // common on a quantised frame, and an unstable sort would order tied
    // candidates by whatever the implementation happened to do — which is
    // exactly the kind of thing that differs between libstdc++ and MSVC and
    // would break reproducibility across platforms.
    std::stable_sort(out.begin(), out.end(),
                     [](const Detection& a, const Detection& b) { return a.snr > b.snr; });

    if (static_cast<int>(out.size()) > params_.max_candidates) {
        out.resize(static_cast<size_t>(params_.max_candidates));
    }
}

}  // namespace sat
