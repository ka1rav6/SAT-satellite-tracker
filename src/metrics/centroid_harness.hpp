// metrics/centroid_harness.hpp — CP 9.2, 9.4, 9.5.
//
// ---------------------------------------------------------------------------
// WHY THIS IS IN metrics/ AND NOT IN perception/centroid/ WITH THE ESTIMATORS
// ---------------------------------------------------------------------------
// It was written there first, and INV-1 rejected it. The harness renders its
// beacons with the simulator's own exact-coverage splat (camera/coverage.hpp),
// and sat_perception may not see sat_camera — the camera knows where every
// emitter is, so a perception module able to include it would be one step from
// reading the answer. cmake/modules.cmake asserts that edge at configure time
// and tools/check_source_invariants.py catches the header-only form, which is
// how this was found: the link check would not have, because a header-only
// include creates no library edge at all.
//
// The split is also right on its own terms. The harness is a MEASURING
// INSTRUMENT — it never ships inside the loop, and it exists to say how
// accurate an estimator is, which is what metrics/ is for.
// perception/centroid/ holds the estimators themselves, which do ship, and
// they stay unable to see the simulator.
//
// CP 9.2  "Accuracy harness: 200 offsets x 6 sizes x 8 SNR bins. Produces the
//          error-vs-offset curve per estimator; the S-shape is visible."
// CP 9.4  "centroid_sigma(), fed into Kalman R. Calibration check over 100k
//          frames shows actual error consistent with claimed sigma."
// CP 9.5  "Theoretical bound computed and plotted against measured."
//
// ---------------------------------------------------------------------------
// WHY THE HARNESS RENDERS ITS OWN BEACONS
// ---------------------------------------------------------------------------
// It would be easier to run the full simulator and read the truth out. It would
// also measure the wrong thing: the simulator's beacon lands at a sub-pixel
// position determined by the motion algebra, so sampling 200 evenly spaced
// phases would need 200 carefully chosen scenarios, and any error in that setup
// would appear as centroid error.
//
// Rendering directly means the true position is an input rather than a
// derivation. The renderer here is the same exact-coverage splat the simulator
// uses (§9.2), so the profile is identical — what differs is only that the
// harness chooses the phase instead of discovering it.

#pragma once

#include "core/rng.hpp"
#include "perception/centroid/bias.hpp"
#include "perception/centroid/estimators.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// The theoretical limit, §10.1.1.
//
//     sigma_centroid >~ w / (2 * SNR)
//
// w is the beacon's width in pixels and SNR the integrated signal-to-noise
// ratio. The design tabulates four points from it — clear ~0.10 px at SNR 50,
// fog + low light + max noise ~0.71 px at SNR 7 — and CP 9.5 asks for measured
// accuracy plotted against it.
//
// It is a BOUND, so being above it is expected and being far below it means the
// measurement is wrong. CP 9.6 asserts within 1.5x of it at every SNR bin,
// which is the useful form: a factor, at every condition, rather than a single
// number at a flattering one.
// ---------------------------------------------------------------------------
[[nodiscard]] double centroid_bound_px(double size_px, double snr) noexcept;

// ---------------------------------------------------------------------------
// One measurement cell.
// ---------------------------------------------------------------------------
struct CentroidAccuracy {
    CentroidKind kind = CentroidKind::WindowedCoM;
    int   size_px = 10;
    float snr     = 20.0f;

    int    samples = 0;
    double rmse_px = 0.0;      ///< over both axes
    double bias_px = 0.0;      ///< mean signed, x axis — the S-curve's DC term
    double p95_px  = 0.0;
    double max_px  = 0.0;

    /// The S-curve itself: error against true sub-pixel phase, both raw.
    std::vector<double> u;         ///< true fractional part, [0, 1)
    std::vector<double> err_x;     ///< signed error in x, pixels

    BiasCoeffs fit{};              ///< a, b from fit_s_curve over (u, err_x)

    /// Fraction of the error's variance the fitted S-curve accounts for.
    ///
    /// The deciding number for whether a cell's coefficients are worth
    /// STORING. At high SNR the S-curve is most of the error and this is large;
    /// at low SNR the random component buries it and a least-squares fit will
    /// still return two numbers — fitted to noise, and applying them makes the
    /// estimate slightly worse. Measured: correcting the SNR 3 to 13 cells cost
    /// 2-5% accuracy before this gate was added.
    double explained = 0.0;

    /// §10.1.1's bound at this cell, and the ratio to it (CP 9.5, CP 9.6).
    [[nodiscard]] double bound_px() const noexcept {
        return centroid_bound_px(size_px, snr);
    }
    [[nodiscard]] double ratio_to_bound() const noexcept {
        const double b = bound_px();
        return b > 0.0 ? rmse_px / b : 0.0;
    }
};

// ---------------------------------------------------------------------------
// CentroidHarness
// ---------------------------------------------------------------------------
struct HarnessParams {
    int  offsets = 200;        ///< §10.1.3 step 1: "200 known sub-pixel offsets"

    /// Canvas size. Large enough for a 20 px beacon's structuring element —
    /// §9.4.2's is 2*size+5, so 45 px at the top of spec row 10's range — plus
    /// margin, because a morphological opening whose kernel is a large
    /// fraction of the image behaves like a global one.
    int  image = 96;

    // -----------------------------------------------------------------------
    // HOW THE SNR IS SET, AND WHY IT IS NOT BY SCALING THE NOISE.
    //
    // §10.1.1's SNR is INTEGRATED over the beacon: total signal over
    // sigma_pixel * sqrt(n_pixels). The obvious way to hit a target is to fix
    // the beacon's amplitude and solve for the noise, and it does not work.
    // With a 10 px beacon at 200 grey levels the total signal is 20,000, so
    // SNR 20 demands a per-pixel sigma of 100 — half the 8-bit range. The
    // frames clip, the beacon disappears into the noise floor, and the harness
    // measures the detector failing rather than the estimator's accuracy. That
    // is exactly what the first version did: 38 px of "centroid error" at
    // SNR 5, and an error uncorrelated between noise realisations, which is
    // the signature of measuring noise rather than bias.
    //
    // So the NOISE is fixed at a realistic level and the beacon's AMPLITUDE is
    // solved for:
    //
    //     amplitude = snr * sigma_pixel / sqrt(n_pixels)
    //
    // which for a 10 px beacon at sigma 10 gives 80 grey levels at SNR 80 and
    // 3 at SNR 3. A beacon three levels above the background IS what SNR 3
    // means, and it stays inside 8 bits.
    // -----------------------------------------------------------------------
    double noise_sigma = 10.0;   ///< per-pixel, grey levels; spec row 22 caps at 20
    double background  = 16.0;   ///< the sensor's black level (degrade/noise.hpp)
    uint64_t seed = 20260914;

    /// Apply the bias table while measuring. The before/after comparison
    /// CP 9.3 asks for is two runs of the harness differing only in this.
    bool correct_bias = false;
    const BiasTable* table = nullptr;
};

/// Measure one (estimator, size, SNR) cell.
[[nodiscard]] CentroidAccuracy measure_cell(CentroidKind kind, int size_px, float snr,
                                            const HarnessParams& p);

/// The whole grid: every estimator x every size x every SNR bin.
[[nodiscard]] std::vector<CentroidAccuracy> measure_grid(const HarnessParams& p);

/// Minimum explained variance for a cell's coefficients to be stored.
///
/// 0.10 is low deliberately: the aim is to exclude cells where the fit is
/// fitting noise, not to demand a good fit. At 10% the curve is clearly
/// present; below it, the measured gain from correcting was negative.
inline constexpr double kMinExplainedVariance = 0.10;

/// Fit a bias table from a grid measured with correct_bias = false.
[[nodiscard]] BiasTable fit_table(const std::vector<CentroidAccuracy>& grid);

}  // namespace sat
