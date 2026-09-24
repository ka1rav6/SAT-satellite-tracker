// degrade/turbulence.cpp — see turbulence.hpp for the physics and the
// stated simplifications. This file is the arithmetic only.

#include "degrade/turbulence.hpp"

#include <algorithm>
#include <cmath>

namespace sat {
namespace {

/// Kolmogorov tilt PSD exponent. Not a tuning constant: it follows from the
/// -11/3 exponent of the Kolmogorov refractive-index spectrum, and it is the
/// number tests/degrade/test_turbulence.cpp fits and asserts.
constexpr double kKolmogorovBeta = 11.0 / 3.0;

}  // namespace

// ===========================================================================
// PowerLawNoise — Kasdin (1995)
// ===========================================================================

void PowerLawNoise::configure(double beta, double sigma) noexcept {
    // Split the exponent so the FIR lands in the range where its coefficients
    // decay fast enough to fit in kTaps taps. See the long comment in
    // turbulence.hpp: the FIR is only accurate for beta' in (-2, 0].
    stages_ = (beta > 0.0)
                  ? std::min(kMaxStages, static_cast<int>(std::ceil(beta / 2.0)))
                  : 0;
    const double fir_beta = beta - 2.0 * static_cast<double>(stages_);

    // h[0] = 1, h[k] = h[k-1] * (k - 1 + beta'/2) / k.
    //
    // The binomial expansion of (1 - z^-1)^(-beta'/2). Filtering white noise
    // with it gives a PSD proportional to |1 - e^(-i w)|^(-beta'), which is
    // f^(-beta') for w << 1 — i.e. for frequencies well below Nyquist, which
    // is the band a camera resolves. A negative beta' is a fractional
    // differentiator and is just as valid here as a positive one.
    const double half_beta = 0.5 * fir_beta;
    h_[0] = 1.0;
    for (size_t k = 1; k < kTaps; ++k) {
        h_[k] = h_[k - 1] * ((static_cast<double>(k) - 1.0 + half_beta)
                             / static_cast<double>(k));
    }

    // The integrators' corner: one cycle per kPolePeriod samples, i.e. a time
    // constant of kPolePeriod/2pi = 163 frames, or 5.4 s at 30 Hz. Below it
    // the spectrum flattens; above it each stage is accurate f^(-2).
    //
    // It is deliberately placed a factor of four BELOW the FIR's own knee
    // rather than on top of it. The leak bends the response for roughly a
    // decade above its corner, so a corner at the edge of the measured band
    // drags the fitted slope up — putting it well below the band leaves the
    // whole resolved range clean. Measured against the analytic response, a
    // corner here costs 0.03 of the exponent; a corner at 1/kTaps cost 0.12.
    //
    // Physically this is outer-scale saturation, and 5.4 s is a defensible
    // outer-scale crossing time. Making it much longer would be closer to
    // pure Kolmogorov but would let the boresight random-walk across a 60 s
    // run in a way that is indistinguishable from platform drift.
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    constexpr double kPolePeriod = 4.0 * static_cast<double>(kTaps);
    pole_ = std::exp(-kTwoPi / kPolePeriod);

    // The cascade's gain on unit white noise: push the FIR's impulse response
    // through the same integrators and sum the squares. Normalising by
    // sqrt(sum g^2) is what makes the configured sigma mean the OUTPUT
    // standard deviation — and the output is what has to match sigma_AoA.
    //
    // The tail runs 32 time constants of the pole past the FIR, which at
    // kTaps = 256 leaves pole^n below 1e-14 and the truncation invisible.
    double sum_sq = 0.0;
    {
        const size_t tail =
            kTaps + static_cast<size_t>(32.0 * kPolePeriod / kTwoPi)
                        * static_cast<size_t>(std::max(1, stages_));
        double state[kMaxStages] = {0.0, 0.0};
        for (size_t n = 0; n < tail; ++n) {
            double g = (n < kTaps) ? h_[n] : 0.0;
            for (int st = 0; st < stages_; ++st) {
                state[st] = pole_ * state[st] + g;
                g = state[st];
            }
            sum_sq += g * g;
        }
    }
    fir_sigma_ = std::sqrt(sum_sq);
    scale_     = (fir_sigma_ > 0.0) ? (sigma / fir_sigma_) : 0.0;

    ring_.fill(0.0);
    head_ = 0;
    acc_.fill(0.0);
    ready_ = true;
}

double PowerLawNoise::next(Pcg32& g) noexcept {
    // ONE draw per call, unconditionally. A draw that depended on `ready_` or
    // on the value being used would desynchronise the stream between two runs
    // that differ only in a consumer, and INV-3 would fail in a way that looks
    // like a physics bug.
    const double w = g.next_normal();
    if (!ready_) return 0.0;

    // Newest sample at head_, walking backwards through history.
    head_ = (head_ == 0) ? (kTaps - 1) : (head_ - 1);
    ring_[head_] = w;

    double acc = 0.0;
    size_t idx = head_;
    for (size_t k = 0; k < kTaps; ++k) {
        acc += h_[k] * ring_[idx];
        idx = (idx + 1 == kTaps) ? 0 : (idx + 1);
    }

    // The integrator chain. stages_ == 0 leaves acc untouched.
    for (int st = 0; st < stages_; ++st) {
        acc_[static_cast<size_t>(st)] = pole_ * acc_[static_cast<size_t>(st)] + acc;
        acc = acc_[static_cast<size_t>(st)];
    }
    return acc * scale_;
}

void PowerLawNoise::reset() noexcept {
    ring_.fill(0.0);
    head_ = 0;
    acc_.fill(0.0);
}

// ===========================================================================
// TurbulenceModel
// ===========================================================================

void TurbulenceModel::build(const TurbulenceParams& p, double camera_hz) {
    params_ = p;
    aoa_    = Angle2{};
    log_i_.fill(0.0);

    if (!p.enabled) {
        // Leave everything zeroed. irradiance_gain() returns 1 and aoa_offset()
        // returns {0,0}, so a disabled model is bit-identical to no model.
        ou_a_ = ou_b_ = log_mu_ = log_sigma_ = 0.0;
        return;
    }

    const double sigma = p.aoa_sigma_urad();
    aoa_x_.configure(kKolmogorovBeta, sigma);
    aoa_y_.configure(kKolmogorovBeta, sigma);

    // --- scintillation ----------------------------------------------------
    // I = exp(X), X ~ N(mu, sigma_X^2). For a log-normal with unit mean and
    // scintillation index sigma_I^2 = Var(I)/E[I]^2:
    //
    //     sigma_X^2 = ln(1 + sigma_I^2),     mu = -sigma_X^2 / 2
    //
    // The mu term is what keeps E[I] = 1. Without it, turning scintillation on
    // would also brighten the beacon, and the ablation would be measuring two
    // things at once.
    if (p.scintillation_index > 0.0) {
        const double var_x = std::log(1.0 + p.scintillation_index);
        log_sigma_ = std::sqrt(var_x);
        log_mu_    = -0.5 * var_x;

        // Atmospheric coherence time tau ~ r0 / V. At r0 = 5 cm and V = 5 m/s
        // that is 10 ms — shorter than a 33 ms frame, so at default parameters
        // the AR(1) is very nearly white, which is physically correct: a 30 Hz
        // camera cannot resolve scintillation. The parameterisation is kept
        // anyway so a slower atmosphere or a faster camera behaves properly
        // rather than being white by construction.
        const double tau_s = (p.wind_ms > 0.0) ? (p.r0_m / p.wind_ms) : 0.0;
        const double dt_s  = (camera_hz > 0.0) ? (1.0 / camera_hz) : 0.0;
        ou_a_ = (tau_s > 0.0 && dt_s > 0.0) ? std::exp(-dt_s / tau_s) : 0.0;
        // Innovation scaled so the stationary variance stays var_x whatever
        // the retention is. This is the standard exact AR(1) discretisation of
        // an OU process, not an Euler step: an Euler step would make the
        // realised variance depend on dt, and the scintillation index would
        // silently stop meaning what it says at a different frame rate.
        ou_b_ = log_sigma_ * std::sqrt(std::max(0.0, 1.0 - ou_a_ * ou_a_));

        // Start each emitter in the stationary distribution rather than at the
        // mean, so frame 0 is not systematically brighter than frame 100.
        log_i_.fill(log_mu_);
    } else {
        ou_a_ = ou_b_ = log_mu_ = log_sigma_ = 0.0;
    }
}

void TurbulenceModel::step(RngSet& rng) noexcept {
    if (!params_.enabled) return;
    Pcg32& g = rng[Stream::Atmosphere];

    // AoA first, then scintillation, always in this order and always the same
    // number of draws per frame. The draw count must not depend on how many
    // emitters happen to be visible, or the stream would desynchronise between
    // two runs of the same scenario that differ only in the search pattern.
    aoa_ = Angle2{aoa_x_.next(g), aoa_y_.next(g)};

    if (log_sigma_ > 0.0) {
        for (size_t i = 0; i < kMaxGraded; ++i) {
            log_i_[i] = log_mu_ + ou_a_ * (log_i_[i] - log_mu_) + ou_b_ * g.next_normal();
        }
    }
}

void TurbulenceModel::reset() noexcept {
    aoa_x_.reset();
    aoa_y_.reset();
    aoa_ = Angle2{};
    log_i_.fill(log_mu_);
}

double TurbulenceModel::irradiance_gain(size_t slot) const noexcept {
    if (!params_.enabled || log_sigma_ <= 0.0 || slot >= kMaxGraded) return 1.0;
    return std::exp(log_i_[slot]);
}

}  // namespace sat
