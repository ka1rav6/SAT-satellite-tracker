// perception/centroid/bias.hpp — CP 9.2 and CP 9.3.
//
// §10.1.3: "S-curve bias correction — THE HIGHEST-LEVERAGE SINGLE ITEM."
//
// ---------------------------------------------------------------------------
// WHAT THE S-CURVE IS
// ---------------------------------------------------------------------------
// A centre-of-mass estimator's error is not random. Plot it against where
// inside a pixel the true centre lies and it traces a repeating S: zero when
// the centre is on a pixel boundary or a pixel centre, largest a quarter of the
// way between. It is a BIAS, not noise — averaging a thousand frames does not
// reduce it — and above about SNR 20 it is larger than the random error, which
// makes it the thing standing between this project and its graded metric.
//
// Three causes, all structural:
//
//   * the finite summation window truncates the profile asymmetrically, so a
//     centre left of a pixel's middle keeps more of its right tail than its
//     left and the estimate is pulled right;
//   * the background subtraction leaves a residual that is itself a function
//     of sub-pixel phase;
//   * the median filter reshapes the profile slightly differently depending on
//     where the peak falls relative to the sampling grid.
//
// ---------------------------------------------------------------------------
// THE MODEL, AND WHY IT IS TWO SINE TERMS
// ---------------------------------------------------------------------------
// §10.1.3 prescribes
//
//     bias(u) ~ a*sin(2*pi*u) + b*sin(4*pi*u)
//
// where u is the estimated fractional part. The form is not arbitrary. The bias
// must be PERIODIC in u with period 1, because the pixel grid is, so it has a
// Fourier series in u. It must also be ODD about u = 0.5 by the symmetry of the
// problem — a beacon displaced left of a pixel centre is the mirror image of
// one displaced right — which kills every cosine term. Two harmonics is what
// the measured curve needs; a third contributes under a thousandth of a pixel.
//
// Fitting two amplitudes rather than storing the sampled curve matters for a
// different reason: the table is compiled in, and 2 doubles per (size, SNR,
// estimator) cell is 50 numbers where a sampled curve would be 200 per cell.
//
// ---------------------------------------------------------------------------
// ONE ITERATION, NOT MORE
// ---------------------------------------------------------------------------
// The correction is applied to the ESTIMATED fractional part, not the true one,
// which is unknown at runtime. That makes it a fixed-point problem, and §10.1.3
// says to iterate once. Once is right: the residual after one pass is roughly
// the square of the original relative error, so a 0.05 px bias becomes ~0.003 px
// and a second pass would chase noise. Iterating to convergence on a noisy
// estimate is how a correction turns into an oscillation.

#pragma once

#include "core/units.hpp"
#include "perception/centroid/estimators.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// The table's axes. §10.1.3: "{6 sizes} x {8 SNR bins} x {4 estimators}".
// ---------------------------------------------------------------------------
inline constexpr int kBiasSizes = 6;
inline constexpr int kBiasSnr   = 8;

/// Beacon sizes the table is measured at, in pixels. Spec row 10 permits 5-20.
inline constexpr std::array<int, kBiasSizes> kBiasSizePx{5, 8, 11, 14, 17, 20};

/// SNR bin edges. Logarithmic, because the interesting behaviour is at the
/// bottom: the difference between SNR 3 and 6 matters far more than between 40
/// and 80, and a linear binning would spend most of its resolution where
/// nothing changes.
inline constexpr std::array<float, kBiasSnr> kBiasSnrCentres{
    3.0f, 5.0f, 8.0f, 13.0f, 20.0f, 32.0f, 50.0f, 80.0f};

[[nodiscard]] int bias_size_bin(int size_px) noexcept;
[[nodiscard]] int bias_snr_bin(float snr) noexcept;

// ---------------------------------------------------------------------------
// BiasCoeffs — one cell of the table.
// ---------------------------------------------------------------------------
struct BiasCoeffs {
    float a = 0.0f;   ///< sin(2*pi*u) amplitude, pixels
    float b = 0.0f;   ///< sin(4*pi*u) amplitude, pixels

    [[nodiscard]] bool measured() const noexcept { return a != 0.0f || b != 0.0f; }
};

/// bias(u) = a sin(2 pi u) + b sin(4 pi u), for u in [0, 1).
[[nodiscard]] double bias_at(const BiasCoeffs& c, double u) noexcept;

// ---------------------------------------------------------------------------
// BiasTable — the compiled-in correction (CP 9.3 step 4).
// ---------------------------------------------------------------------------
class BiasTable {
public:
    /// The table shipped with the build, measured by CP 9.2's harness.
    [[nodiscard]] static const BiasTable& builtin() noexcept;

    [[nodiscard]] const BiasCoeffs& at(CentroidKind k, int size_bin, int snr_bin) const noexcept;
    void set(CentroidKind k, int size_bin, int snr_bin, BiasCoeffs c) noexcept;

    /// Correct one axis. `v` is the raw estimate in pixels; the correction is
    /// looked up at its own fractional part and subtracted, once.
    [[nodiscard]] double correct(CentroidKind k, int size_px, float snr, double v) const noexcept;

    /// Correct both axes of an estimate.
    [[nodiscard]] Pixel2 correct(CentroidKind k, int size_px, float snr, Pixel2 p) const noexcept;

    /// Emit the table as a C++ source fragment, for regenerating the built-in
    /// one after a measurement run. Keeping the generator in the program rather
    /// than in a script means the numbers cannot be produced by a different
    /// code path from the one that uses them.
    [[nodiscard]] std::string to_source() const;

private:
    std::array<std::array<std::array<BiasCoeffs, kBiasSnr>, kBiasSizes>,
               static_cast<size_t>(CentroidKind::kCount)> t_{};
};

// ---------------------------------------------------------------------------
// fit_s_curve — CP 9.3 step 3.
//
// Least squares for (a, b) in bias(u) = a sin(2 pi u) + b sin(4 pi u), given
// samples of (u, error). Linear in the coefficients, so this is a 2x2 normal
// equation and not an optimiser.
// ---------------------------------------------------------------------------
[[nodiscard]] BiasCoeffs fit_s_curve(std::span<const double> u,
                                     std::span<const double> error) noexcept;

}  // namespace sat
