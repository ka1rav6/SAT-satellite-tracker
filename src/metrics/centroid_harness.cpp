// metrics/centroid_harness.cpp

#include "metrics/centroid_harness.hpp"

#include "camera/coverage.hpp"
#include "perception/median.hpp"
#include "perception/morphology.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

double centroid_bound_px(double size_px, double snr) noexcept {
    if (!(snr > 0.0)) return 0.0;
    return size_px / (2.0 * snr);
}

namespace {

/// Render a beacon at a known sub-pixel position using the SAME exact-coverage
/// splat the simulator uses (§9.2), so the profile is identical and only the
/// choice of phase differs.
void render(std::vector<float>& radiance, int w, int h,
            Pixel2 centre, double size_px, double intensity) {
    std::fill(radiance.begin(), radiance.end(), 0.0f);
    const int r = static_cast<int>(std::ceil(size_px)) + 2;
    const int x0 = std::max(0, static_cast<int>(centre.x) - r);
    const int x1 = std::min(w - 1, static_cast<int>(centre.x) + r);
    const int y0 = std::max(0, static_cast<int>(centre.y) - r);
    const int y1 = std::min(h - 1, static_cast<int>(centre.y) + r);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const double cov = square_coverage(x, y, centre.x, centre.y, size_px);
            radiance[static_cast<size_t>(y) * w + x] += static_cast<float>(cov * intensity);
        }
    }
}

/// The beacon amplitude that produces a given INTEGRATED SNR (§10.1.1) at a
/// fixed per-pixel noise level. See HarnessParams for why it is solved this way
/// round rather than by scaling the noise.
///
///     snr = (amplitude * n) / (sigma * sqrt(n))  =>  amplitude = snr * sigma / sqrt(n)
double amplitude_for_snr(double snr, double sigma_pixel, double n_pixels) noexcept {
    if (!(n_pixels > 0.0)) return 0.0;
    return snr * sigma_pixel / std::sqrt(n_pixels);
}

/// Add Gaussian noise and quantise to 8 bits, exactly as the sensor does.
void degrade(std::span<const float> radiance, std::span<uint8_t> out,
             double background, double sigma_pixel, Pcg32& rng) {
    for (size_t i = 0; i < out.size(); ++i) {
        const double v = background + radiance[i] + rng.next_normal() * sigma_pixel;
        out[i] = static_cast<uint8_t>(std::clamp(v + 0.5, 0.0, 255.0));
    }
}

}  // namespace

CentroidAccuracy measure_cell(CentroidKind kind, int size_px, float snr,
                              const HarnessParams& p) {
    CentroidAccuracy acc;
    acc.kind = kind;
    acc.size_px = size_px;
    acc.snr = snr;

    const int w = p.image, h = p.image;
    const size_t n = static_cast<size_t>(w) * h;
    std::vector<float>   radiance(n);
    std::vector<uint8_t> raw(n), filtered(n);
    std::vector<int16_t> tophat(n);

    MorphWorkspace morph;
    std::vector<uint8_t> ma(n), mb(n), mc(n), ms(n);
    morph.a = ma; morph.b = mb; morph.c = mc; morph.scratch = ms;

    RngSet rng(p.seed + static_cast<uint64_t>(size_px) * 1000
               + static_cast<uint64_t>(snr * 10.0f));
    Pcg32& g = rng[Stream::ReadNoise];

    acc.u.reserve(static_cast<size_t>(p.offsets));
    acc.err_x.reserve(static_cast<size_t>(p.offsets));

    const int se = structuring_element_size(size_px);
    const int win = std::max(2, size_px / 2 + 1);

    double sum_sq = 0.0, sum_dx = 0.0, worst = 0.0;
    std::vector<double> mags;
    mags.reserve(static_cast<size_t>(p.offsets));

    for (int i = 0; i < p.offsets; ++i) {
        // 200 offsets across ONE pixel (§10.1.3 step 1). The phase is what the
        // S-curve is a function of, so it is swept densely and the integer part
        // is held fixed — moving the beacon across the image instead would
        // confound the phase with position-dependent effects.
        const double u = static_cast<double>(i) / p.offsets;
        const Pixel2 truth{w * 0.5 + u, h * 0.5 + 0.5};

        const double area = static_cast<double>(size_px) * size_px;
        const double amplitude = amplitude_for_snr(snr, p.noise_sigma, area);
        render(radiance, w, h, truth, size_px, amplitude);
        degrade(radiance, raw, p.background, p.noise_sigma, g);

        // The same front end the pipeline runs (§9.4 B6-B7), because the
        // estimators are defined on the top-hat and because the median filter
        // is itself one of the S-curve's causes.
        median_3x3(raw, filtered, w, h);
        top_hat(filtered, tophat, w, h, se, morph);

        // ------------------------------------------------------------------
        // THE SEARCH IS RESTRICTED TO THE REGION THE HARNESS RENDERED INTO.
        //
        // This measures the ESTIMATOR given a detection, not the detector.
        // Stage 5 tests detection, against clutter and impulse noise and fog,
        // and does it properly with CFAR and connected components. Repeating a
        // worse version of that here would mean CP 9.2's numbers moved when
        // detection changed, which is the opposite of what a calibration
        // harness is for.
        //
        // It is not privileged information: the harness CHOSE where to render,
        // so the neighbourhood is its own setup rather than something read out
        // of a simulator. INV-1 is about the tracker, and this is not the
        // tracker — it is a measuring instrument that never ships in the loop.
        //
        // Without the restriction the measurement was nonsense below about
        // amplitude 60: a median-filtered sigma-10 noise field has a top-hat
        // maximum of roughly 28 grey levels over a 96x96 image, which rivals a
        // faint beacon outright, so the global maximum was often noise and the
        // "blob" was pixels scattered across the whole frame. It showed up as a
        // 13x jump in error between SNR 50 and SNR 80 — a threshold effect
        // pretending to be a trend.
        // ------------------------------------------------------------------
        const int rx = size_px + 8;
        const int cxi = w / 2, cyi = h / 2;
        const int rx0 = std::max(0, cxi - rx), rx1 = std::min(w - 1, cxi + rx);
        const int ry0 = std::max(0, cyi - rx), ry1 = std::min(h - 1, cyi + rx);

        // The brightest pixel in that region, used ONLY to set a threshold —
        // never as the window's centre.
        //
        // Spec row 9's default beacon is a SQUARE, so its top-hat is flat
        // across the whole shape and argmax is degenerate: it returns whichever
        // of a hundred equal pixels noise happens to favour, which is usually a
        // corner. Centring a window there cuts off most of the beacon and the
        // estimate is pulled several pixels toward that corner.
        //
        // The symptom was unmistakable once measured: 1.4 px RMSE at SNR 80,
        // and — the giveaway — an error UNCORRELATED between two noise
        // realisations at the same sub-pixel phase. A systematic S-curve is
        // perfectly correlated; a wandering argmax is not.
        //
        // So the window is centred on the blob's centre of mass, which is what
        // perception/pipeline.cpp does for the same reason. A rough first
        // estimate that is robust beats an exact one that is degenerate.
        // ------------------------------------------------------------------
        int16_t best = 0;
        for (int y = ry0; y <= ry1; ++y) {
            for (int x = rx0; x <= rx1; ++x) {
                best = std::max(best, tophat[static_cast<size_t>(y) * w + x]);
            }
        }

        // A BlobAccum over the thresholded region, for the CoM estimator.
        //
        // Half the peak rather than a quarter: the beacon is a flat-topped
        // square, so half-max sits on its shoulder where the profile is
        // steepest, and the contour is least sensitive to noise there. A
        // quarter-max contour runs through the top-hat's shallow skirt and
        // wanders by a pixel or more between frames.
        BlobAccum b{};
        b.x0 = static_cast<int16_t>(w); b.y0 = static_cast<int16_t>(h);
        for (int y = ry0; y <= ry1; ++y) {
            for (int x = rx0; x <= rx1; ++x) {
                const int16_t v = tophat[static_cast<size_t>(y) * w + x];
                if (v <= best / 2) continue;
                const double wgt = v;
                b.n++; b.sw += wgt; b.swx += wgt * x; b.swy += wgt * y;
                b.x0 = std::min<int16_t>(b.x0, static_cast<int16_t>(x));
                b.x1 = std::max<int16_t>(b.x1, static_cast<int16_t>(x));
                b.y0 = std::min<int16_t>(b.y0, static_cast<int16_t>(y));
                b.y1 = std::max<int16_t>(b.y1, static_cast<int16_t>(y));
                b.peak = std::max(b.peak, static_cast<float>(v));
            }
        }

        const Pixel2 seed = b.centroid();
        Pixel2 est = centroid_estimate(kind, b, tophat, {}, w, h, seed, win);
        if (p.correct_bias && p.table) {
            est = p.table->correct(kind, size_px, snr, est);
        }

        const double dx = est.x - truth.x;
        const double dy = est.y - truth.y;
        acc.u.push_back(u);
        acc.err_x.push_back(dx);

        const double mag = std::hypot(dx, dy);
        sum_sq += dx * dx + dy * dy;
        sum_dx += dx;
        worst = std::max(worst, mag);
        mags.push_back(mag);
    }

    acc.samples = p.offsets;
    // RMSE over both axes: sum of squared errors divided by the number of
    // SAMPLES, not of components — this is the RMS of the 2-D error magnitude,
    // which is what §13.1's centroiding_error is.
    acc.rmse_px = std::sqrt(sum_sq / std::max(1, p.offsets));
    acc.bias_px = sum_dx / std::max(1, p.offsets);
    acc.max_px  = worst;
    std::sort(mags.begin(), mags.end());
    acc.p95_px = mags.empty() ? 0.0
        : mags[std::min(mags.size() - 1,
                        static_cast<size_t>(std::ceil(0.95 * mags.size())) - 1)];
    acc.fit = fit_s_curve(acc.u, acc.err_x);

    // How much of the error the fitted curve actually accounts for. 1 - SSres
    // / SStot about zero, not about the mean: the model has no constant term
    // by construction (the bias is odd about the pixel centre), so measuring
    // against the mean would credit it for a DC offset it cannot represent.
    double ss_tot = 0.0, ss_res = 0.0;
    for (size_t i = 0; i < acc.u.size(); ++i) {
        const double r = acc.err_x[i] - bias_at(acc.fit, acc.u[i]);
        ss_tot += acc.err_x[i] * acc.err_x[i];
        ss_res += r * r;
    }
    acc.explained = ss_tot > 0.0 ? (1.0 - ss_res / ss_tot) : 0.0;
    return acc;
}

std::vector<CentroidAccuracy> measure_grid(const HarnessParams& p) {
    std::vector<CentroidAccuracy> out;
    out.reserve(static_cast<size_t>(CentroidKind::kCount) * kBiasSizes * kBiasSnr);
    for (uint8_t ki = 0; ki < static_cast<uint8_t>(CentroidKind::kCount); ++ki) {
        const auto k = static_cast<CentroidKind>(ki);
        // Learned is WindowedCoM until Stage 11 (INV-7). Measuring it would
        // duplicate a row and imply a capability that does not exist.
        if (k == CentroidKind::Learned) continue;
        // MatchedPeak needs a response map the harness does not build; it is
        // measured in the pipeline tests instead. Silently producing a
        // WindowedCoM row labelled MatchedPeak would be worse than omitting it.
        if (k == CentroidKind::MatchedPeak) continue;
        for (int s : kBiasSizePx) {
            for (float snr : kBiasSnrCentres) {
                out.push_back(measure_cell(k, s, snr, p));
            }
        }
    }
    return out;
}

BiasTable fit_table(const std::vector<CentroidAccuracy>& grid) {
    BiasTable t;
    for (const CentroidAccuracy& c : grid) {
        // ONLY STORE A CELL WHOSE CURVE IS ACTUALLY RESOLVABLE.
        //
        // Least squares always returns two numbers. At low SNR they are fitted
        // to noise, and subtracting them makes the estimate worse — measured
        // at 2-5% across the SNR 3 to 13 cells. A correction that is not
        // supported by the data is exactly the "wrong correction is worse than
        // none" case bias.hpp is built around, and leaving the cell empty is
        // how the table says so: BiasTable::correct() passes an unmeasured
        // cell through untouched.
        //
        // The threshold is on explained variance rather than on SNR, because
        // what matters is whether the curve is visible in THIS cell — a large
        // beacon at moderate SNR can resolve it where a small one cannot.
        if (c.explained < kMinExplainedVariance) continue;
        t.set(c.kind, bias_size_bin(c.size_px), bias_snr_bin(c.snr), c.fit);
    }
    return t;
}

}  // namespace sat
