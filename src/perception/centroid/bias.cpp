// perception/centroid/bias.cpp

#include "perception/centroid/bias.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace sat {

int bias_size_bin(int size_px) noexcept {
    // Nearest centre. A beacon of 6 px belongs with the 5 px measurements, not
    // with the 8 px ones, and clamping keeps anything outside spec row 10's
    // 5-20 range in the nearest bin rather than out of the table.
    int best = 0;
    int best_d = std::abs(size_px - kBiasSizePx[0]);
    for (int i = 1; i < kBiasSizes; ++i) {
        const int d = std::abs(size_px - kBiasSizePx[static_cast<size_t>(i)]);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

int bias_snr_bin(float snr) noexcept {
    // Nearest in LOG, matching how the centres were chosen. Nearest in linear
    // would put SNR 4 with the 5 bin and SNR 6.4 with the 8 bin, which is the
    // wrong side of both — the bins are ratios, not intervals.
    const double s = std::log(std::max(snr, 0.1f));
    int best = 0;
    double best_d = std::fabs(s - std::log(kBiasSnrCentres[0]));
    for (int i = 1; i < kBiasSnr; ++i) {
        const double d = std::fabs(s - std::log(kBiasSnrCentres[static_cast<size_t>(i)]));
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

double bias_at(const BiasCoeffs& c, double u) noexcept {
    // Wrap into [0, 1). A negative estimate's fractional part must still be a
    // phase, and std::fmod alone would give a negative one.
    u -= std::floor(u);
    constexpr double tau = 6.283185307179586;
    return c.a * std::sin(tau * u) + c.b * std::sin(2.0 * tau * u);
}

BiasCoeffs fit_s_curve(std::span<const double> u, std::span<const double> error) noexcept {
    // Least squares on the two basis functions. Linear in (a, b), so this is a
    // 2x2 normal equation:  [[S11,S12],[S12,S22]] [a,b]^T = [T1,T2]^T.
    constexpr double tau = 6.283185307179586;
    double s11 = 0, s12 = 0, s22 = 0, t1 = 0, t2 = 0;
    const size_t n = std::min(u.size(), error.size());
    for (size_t i = 0; i < n; ++i) {
        const double f1 = std::sin(tau * u[i]);
        const double f2 = std::sin(2.0 * tau * u[i]);
        s11 += f1 * f1; s12 += f1 * f2; s22 += f2 * f2;
        t1  += f1 * error[i]; t2 += f2 * error[i];
    }
    const double det = s11 * s22 - s12 * s12;
    // A degenerate system means the samples do not span the phase — too few
    // offsets, or all at the same one. Returning zeros is correct: an unmeasured
    // cell must not correct anything, and BiasCoeffs::measured() reports it.
    if (std::fabs(det) < 1e-12) return BiasCoeffs{};
    return BiasCoeffs{
        static_cast<float>((t1 * s22 - t2 * s12) / det),
        static_cast<float>((s11 * t2 - s12 * t1) / det)};
}

const BiasCoeffs& BiasTable::at(CentroidKind k, int size_bin, int snr_bin) const noexcept {
    static const BiasCoeffs kZero{};
    const auto ki = static_cast<size_t>(k);
    if (ki >= t_.size()) return kZero;
    if (size_bin < 0 || size_bin >= kBiasSizes) return kZero;
    if (snr_bin  < 0 || snr_bin  >= kBiasSnr)   return kZero;
    return t_[ki][static_cast<size_t>(size_bin)][static_cast<size_t>(snr_bin)];
}

void BiasTable::set(CentroidKind k, int size_bin, int snr_bin, BiasCoeffs c) noexcept {
    const auto ki = static_cast<size_t>(k);
    if (ki >= t_.size()) return;
    if (size_bin < 0 || size_bin >= kBiasSizes) return;
    if (snr_bin  < 0 || snr_bin  >= kBiasSnr)   return;
    t_[ki][static_cast<size_t>(size_bin)][static_cast<size_t>(snr_bin)] = c;
}

double BiasTable::correct(CentroidKind k, int size_px, float snr, double v) const noexcept {
    const BiasCoeffs& c = at(k, bias_size_bin(size_px), bias_snr_bin(snr));
    // An unmeasured cell corrects nothing. That is the important case: a table
    // with holes must degrade to the uncorrected estimator, not to a random
    // shift, because a wrong correction is worse than none.
    if (!c.measured()) return v;

    // ONE iteration (§10.1.3 step 5). The bias is a function of the TRUE
    // fractional part, which is unknown; evaluating at the estimate and
    // subtracting once gets most of the way, and the residual is second order.
    const double u = v - std::floor(v);
    return v - bias_at(c, u);
}

Pixel2 BiasTable::correct(CentroidKind k, int size_px, float snr, Pixel2 p) const noexcept {
    return Pixel2{correct(k, size_px, snr, p.x), correct(k, size_px, snr, p.y)};
}

namespace {

/// The C++ ENUMERATOR name, which is not the display name.
///
/// centroid_kind_name returns "com" for logs and config files; the generated
/// source needs "CoM". Emitting the display name produced a table that did not
/// compile — caught immediately, but only because the table is compiled in
/// rather than loaded at runtime, which is one more argument for doing it that
/// way.
const char* enumerator_name(CentroidKind k) noexcept {
    switch (k) {
        case CentroidKind::CoM:         return "CoM";
        case CentroidKind::WindowedCoM: return "WindowedCoM";
        case CentroidKind::SurfaceFit:  return "SurfaceFit";
        case CentroidKind::MatchedPeak: return "MatchedPeak";
        case CentroidKind::Learned:     return "Learned";
        case CentroidKind::kCount:      break;
    }
    return "CoM";
}

}  // namespace

std::string BiasTable::to_source() const {
    std::string s =
        "// Generated by `sat-tracker --calibrate-centroid`. Do not edit by hand.\n"
        "// Rows: estimator, then beacon size (5,8,11,14,17,20 px), then SNR bin\n"
        "// (3,5,8,13,20,32,50,80). Each cell is {a, b} of\n"
        "//     bias(u) = a sin(2 pi u) + b sin(4 pi u)\n"
        "// in pixels. See design section 10.1.3.\n";
    char b[256];
    for (uint8_t ki = 0; ki < static_cast<uint8_t>(CentroidKind::kCount); ++ki) {
        const auto k = static_cast<CentroidKind>(ki);
        std::snprintf(b, sizeof b, "// --- %s ---\n", centroid_kind_name(k));
        s += b;
        for (int si = 0; si < kBiasSizes; ++si) {
            for (int ni = 0; ni < kBiasSnr; ++ni) {
                const BiasCoeffs& c = at(k, si, ni);
                if (!c.measured()) continue;
                std::snprintf(b, sizeof b,
                              "    t.set(CentroidKind::%s, %d, %d, {%+.6ff, %+.6ff});\n",
                              enumerator_name(k), si, ni,
                              static_cast<double>(c.a), static_cast<double>(c.b));
                s += b;
            }
        }
    }
    return s;
}

// ---------------------------------------------------------------------------
// builtin — the measured table.
//
// Populated by bias_table_generated.inc, which `--calibrate-centroid` writes.
// Kept as an include rather than pasted here so that regenerating it is a
// single-file diff and the surrounding reasoning is not churned by numbers.
// ---------------------------------------------------------------------------
const BiasTable& BiasTable::builtin() noexcept {
    static const BiasTable table = [] {
        BiasTable t;
#include "perception/centroid/bias_table_generated.inc"
        return t;
    }();
    return table;
}

}  // namespace sat
