// degrade/turbulence.hpp — atmospheric turbulence (audit P2-1, roadmap Phase 3).
//
// ---------------------------------------------------------------------------
// WHY THIS FILE EXISTS
// ---------------------------------------------------------------------------
// PS 26169 is a free-space optical communication problem, and the background
// of an FSOC problem statement is *literally about atmospheric propagation*.
// Until this file existed, `Atmosphere` was a five-value enum that applied an
// affine contrast/brightness change to the frame:
//
//     I <- alpha*I + beta          (design §9.3, spec row 24)
//
// That is a PHOTOMETRIC model. "Fog" dimmed the image and nothing else. It is
// a defensible reading of row 24 — which does say "user-defined reduction in
// contrast and brightness" — but it is not a TURBULENCE model, and a Dept. of
// Space evaluator will ask about the two effects that actually dominate an
// FSOC link budget:
//
//   1. ANGLE-OF-ARRIVAL JITTER (beam wander / image dancing). Refractive index
//      fluctuations tilt the incoming wavefront, so the apparent direction of
//      the beacon moves. This is a POINTING disturbance, not an image effect.
//   2. SCINTILLATION. The same fluctuations focus and defocus the beam, so the
//      received irradiance fluctuates. This is what makes an FSOC link drop
//      out, and it is why a tracker must survive a beacon that fades.
//
// Both are now modelled here, parameterised the way the literature and an
// evaluator would expect: by the Fried parameter r0 and the scintillation
// index sigma_I^2.
//
// ---------------------------------------------------------------------------
// WHERE EACH EFFECT IS APPLIED — AND WHY THAT MATTERS
// ---------------------------------------------------------------------------
// AoA jitter is added to the TRUE BORESIGHT, alongside row 23's vibration
// jitter and row 25's platform motion, by DisturbanceGenerator. This is the
// same argument disturbance.hpp makes at length: a disturbance that moves
// where the camera is POINTING must be modelled at the boresight, because
//
//   * it is physically what happens, and
//   * applying it as a pixel shift would leave the commanded boresight equal
//     to the true one, which silently hands the tracker its own pointing
//     error and makes the problem easier than it is.
//
// Scintillation multiplies the BEACON'S IRRADIANCE before the splat, so it
// flows through the exposure integration, the damage chain, the SNR gate and
// the detector exactly as a real irradiance fluctuation would. It is not a
// post-hoc brightness knob.
//
// ---------------------------------------------------------------------------
// THE TEMPORAL SPECTRUM — THE PART THAT IS EASY TO FAKE AND EASY TO CATCH
// ---------------------------------------------------------------------------
// Getting the VARIANCE right is easy. Getting the SPECTRUM right is the part
// that distinguishes a turbulence model from "Gaussian noise with a physics
// word in the variable name", and it is the part an evaluator can check.
//
// Kolmogorov turbulence gives the angle-of-arrival (wavefront tilt) a temporal
// power spectrum that falls as
//
//     S(f) ~ f^(-11/3)
//
// through the inertial subrange. That exponent is not a free parameter: it
// follows from the -11/3 exponent of the Kolmogorov refractive-index spectrum,
// and it is the single number that says whether the model is real.
//
// It is synthesised here with Kasdin's method (N. J. Kasdin, "Discrete
// simulation of colored noise and stochastic processes and 1/f^alpha power law
// noise generation", Proc. IEEE 83(5), 1995): white noise through an FIR whose
// coefficients are the binomial expansion of (1 - z^-1)^(-beta/2),
//
//     h[0] = 1,    h[k] = h[k-1] * (k - 1 + beta/2) / k
//
// which produces f^(-beta) exactly, for f well above the knee the FIR's finite
// length imposes. It is a fixed-length ring buffer of white samples: streaming,
// allocation-free (INV-4), and bit-deterministic (INV-3). tests/degrade/
// test_turbulence.cpp fits the slope of the synthesised PSD and asserts it is
// -11/3 to within a tolerance the test states.
//
// ---------------------------------------------------------------------------
// STATED SIMPLIFICATIONS — read this before defending the model
// ---------------------------------------------------------------------------
// These are deliberate, and saying so is worth more than pretending otherwise:
//
//   * The FIR's finite length L puts a knee at roughly f_camera/L, below which
//     the synthesised spectrum FLATTENS rather than continuing as f^(-11/3).
//     Physically this is outer-scale saturation — a real effect, and the
//     reason the von Karman spectrum exists — but the true sub-knee slope is
//     f^(-2/3), not flat. Above the knee, which is the band this model is for
//     and the band the test asserts, the spectrum is correct.
//   * The two axes are independent. Real AoA tilt has a small cross-axis
//     correlation that depends on the wind direction; it is second-order and
//     is not modelled.
//   * Scintillation is applied per GRADED EMITTER (beacon and decoy), not to
//     clutter. Sources separated by more than the isoplanatic angle (~10 urad)
//     scintillate independently, and every emitter on a 12.5 deg screen is far
//     past that, so a single common gain would be WRONG. Drawing 120
//     independent clutter gains would cost RNG draws and change clutter
//     statistics for no gain in realism, since clutter is scenery rather than
//     a propagating beam. Beacon and decoy get independent draws, which is the
//     case that matters: the decoy must not fade in step with the target.
//   * Aperture averaging of scintillation is not applied; sigma_I^2 is taken
//     as the received, aperture-averaged value the user configures. This makes
//     the knob mean what its name says rather than hiding a lens diameter in
//     it.
//
// ---------------------------------------------------------------------------
// OFF BY DEFAULT
// ---------------------------------------------------------------------------
// `enabled` defaults to false, so every existing scenario, every committed
// number and every historical run is bit-identical with this file present.
// Turbulence is an OPT-IN axis, added as its own `[atmosphere.turbulence]`
// block. INV-3 is checked both ways by the tests.

#pragma once

#include "core/rng.hpp"
#include "core/units.hpp"
#include "scenario/scenario.hpp"

#include <array>
#include <cstddef>

namespace sat {

// ---------------------------------------------------------------------------
// PowerLawNoise — f^(-beta) synthesiser.
//
// Fixed-size, allocation-free in the frame loop, and deterministic given the
// stream. Exposed as its own type so the spectrum can be tested on its own,
// without a scenario, a camera or a beacon in the way.
//
// ---------------------------------------------------------------------------
// WHY IT IS A CASCADE AND NOT JUST KASDIN'S FIR
// ---------------------------------------------------------------------------
// Kasdin's recursion produces f^(-beta) from white noise through an FIR whose
// coefficients are the binomial expansion of (1 - z^-1)^(-beta/2):
//
//     h[0] = 1,    h[k] = h[k-1] * (k - 1 + beta/2) / k
//
// Asymptotically h[k] ~ k^(beta/2 - 1), and that is the whole story of this
// class. The kernel has to DECAY inside kTaps taps or the truncation, not the
// physics, sets the spectrum:
//
//     beta = 11/3   h[k] ~ k^(+0.83)   GROWS. A 256-tap FIR measured
//                                      f^(-1.99) — the truncation's own
//                                      shape, nothing to do with turbulence.
//     beta =  5/3   h[k] ~ k^(-0.17)   decays far too slowly; 256 taps is
//                                      still nearly a boxcar and measured
//                                      f^(-2.41) once integrated.
//     beta = -1/3   h[k] ~ k^(-1.17)   decays fast; converges well inside
//                                      256 taps. THIS is the usable range.
//
// Both failures above were caught by the slope test below before the model was
// believed, which is exactly what that test is for.
//
// So the exponent is split across a fast-decaying FIR and an integer number of
// one-pole integrators, each of which is exactly f^(-2) above its corner:
//
//     m     = ceil(beta / 2)                 number of integrator stages
//     beta' = beta - 2m   in (-2, 0]         what is left for the FIR
//
//     f^(-beta)  =  f^(-beta')  x  ( f^(-2) )^m
//
// At Kolmogorov's 11/3 that is m = 2 and beta' = -1/3: a fractional
// DIFFERENTIATOR, whose coefficients decay as k^(-7/6), followed by two
// integrators. Every stage is then in the regime where it is accurate.
//
// The integrators are LEAKY (pole slightly inside the unit circle) rather than
// plain running sums: a pure integrator is a random walk whose variance grows
// without bound, and a 60 s run would drift off the screen. Each leak's corner
// sits at the FIR's own truncation knee, so every stage flattens at the same
// frequency and the composite is a clean f^(-beta) above it and flat below.
//
// Because the cascade's impulse response is no longer just h[], the
// normalisation constant is computed numerically at configure() time by
// pushing the FIR's impulse response through the same integrators. configure()
// is not called inside the frame loop, so its one temporary is not an INV-4
// concern; next() allocates nothing.
// ---------------------------------------------------------------------------
class PowerLawNoise {
public:
    /// FIR length. 256 taps at 30 Hz puts the knee near 0.12 Hz, which leaves
    /// the whole 0.5-15 Hz band a 30 Hz camera can resolve inside the
    /// inertial range. Larger costs a multiply-add per tap per frame and buys
    /// spectrum below a frequency a 60 s run cannot resolve anyway.
    static constexpr size_t kTaps = 256;

    /// Integrator stages available. Two covers every exponent up to f^(-4),
    /// which is past anything atmospheric: Kolmogorov tilt is 11/3.
    static constexpr int kMaxStages = 2;

    /// beta is the PSD exponent: S(f) ~ f^(-beta). Kolmogorov tilt is 11/3.
    /// Any beta >= 2 takes the cascade described above; below 2 the FIR alone
    /// is stable and the integrator is bypassed.
    void configure(double beta, double sigma) noexcept;

    /// One sample. Draws exactly one normal from `g` per call, always, so the
    /// stream advances identically whether or not the caller uses the value.
    [[nodiscard]] double next(Pcg32& g) noexcept;

    void reset() noexcept;

    /// The unit-white-input standard deviation of the whole cascade. The
    /// scale factor applied to reach the configured sigma is 1/this.
    [[nodiscard]] double fir_sigma() const noexcept { return fir_sigma_; }

private:
    std::array<double, kTaps> h_{};      ///< FIR coefficients, h[0] = 1
    std::array<double, kTaps> ring_{};   ///< past white samples, newest at head_
    size_t head_      = 0;
    double scale_     = 0.0;             ///< sigma / fir_sigma_
    double fir_sigma_ = 1.0;
    double pole_      = 0.0;             ///< leaky-integrator retention
    int    stages_    = 0;               ///< number of integrator stages, 0..kMaxStages
    std::array<double, kMaxStages> acc_{};  ///< integrator states
    bool   ready_     = false;
};

// ---------------------------------------------------------------------------
// TurbulenceModel — the two effects, driven once per camera frame.
//
// Both AoA and scintillation are frame-rate phenomena here: the camera is what
// samples them, and a 300 Hz truth tick would make the synthesised spectrum
// ten times wider than the band the camera can actually resolve.
// ---------------------------------------------------------------------------
class TurbulenceModel {
public:
    /// Emitters that get their own independent scintillation state. Matches
    /// FrameTruth's graded-target capacity: beacon plus decoys.
    static constexpr size_t kMaxGraded = 8;

    void build(const TurbulenceParams& p, double camera_hz);

    /// Advance one CAMERA frame. Draws from Stream::Atmosphere, which
    /// core/rng.hpp reserved for exactly this ("scintillation within a weather
    /// mode") and which nothing else uses, so turbulence cannot perturb the
    /// shot-noise or read-noise streams and break an ablation.
    void step(RngSet& rng) noexcept;

    void reset() noexcept;

    [[nodiscard]] bool enabled() const noexcept { return params_.enabled; }

    /// Boresight offset from angle-of-arrival tilt, microradians. Zero when
    /// disabled. Added to the true boresight, never to the pixels.
    [[nodiscard]] Angle2 aoa_offset() const noexcept { return aoa_; }

    /// Multiplicative irradiance gain for graded emitter `slot`, mean 1.
    /// Returns exactly 1.0 when disabled, when the scintillation index is 0,
    /// or when `slot` is past kMaxGraded — so the caller never needs a branch.
    [[nodiscard]] double irradiance_gain(size_t slot) const noexcept;

    [[nodiscard]] const TurbulenceParams& params() const noexcept { return params_; }

private:
    TurbulenceParams params_{};
    PowerLawNoise    aoa_x_{}, aoa_y_{};
    Angle2           aoa_{};

    // Scintillation is log-normal in irradiance, i.e. NORMAL in log-irradiance,
    // and temporally correlated with the atmosphere's coherence time. The log
    // variable is therefore an Ornstein-Uhlenbeck (AR(1)) process, which is the
    // simplest process with the right stationary distribution AND a finite
    // correlation time. Drawing an independent log-normal every frame would
    // give the right histogram and a white spectrum, which is exactly the
    // "right variance, wrong spectrum" failure this file exists to avoid.
    std::array<double, kMaxGraded> log_i_{};
    double ou_a_     = 0.0;   ///< AR(1) retention exp(-dt/tau)
    double ou_b_     = 0.0;   ///< innovation scale, keeps the stationary variance
    double log_mu_   = 0.0;   ///< -sigma_X^2/2, so E[exp(X)] = 1
    double log_sigma_ = 0.0;  ///< sqrt(ln(1 + sigma_I^2))
};

}  // namespace sat
