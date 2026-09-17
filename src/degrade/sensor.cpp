#include "degrade/sensor.hpp"

#include "camera/splat.hpp"          // quantise_u8
#include "degrade/fast_normal.hpp"
#include "degrade/sensor_simd.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

void SensorChain::build(const Scenario& sc, int width, int height, RngSet& rng) {
    // INV-8. Decided once, here, from the scenario's own answer.
    enabled_    = sc.damage_enabled();
    atmosphere_ = sc.atmosphere;
    noise_      = NoiseParams::from_scenario(sc);

    if (enabled_) {
        fixed_.build(width, height, noise_.prnu_sigma, noise_.fpn_sigma,
                     noise_.hot_pixels, noise_.dead_pixels, rng);
    }
}

void SensorChain::configure(const NoiseParams& np, Atmosphere atm, bool enabled,
                            int width, int height, RngSet& rng) {
    enabled_    = enabled;
    atmosphere_ = atm;
    noise_      = np;
    if (enabled_) {
        fixed_.build(width, height, noise_.prnu_sigma, noise_.fpn_sigma,
                     noise_.hot_pixels, noise_.dead_pixels, rng);
    }
}

// ---------------------------------------------------------------------------
// apply — design §9.3's chain, in one pass.
//
// ---------------------------------------------------------------------------
// WHY ONE PASS
// ---------------------------------------------------------------------------
// This used to be five loops: atmosphere, shot noise, read noise, fixed
// pattern, quantise. Each one read and wrote the whole 1.2 MB float buffer, so
// the chain moved about 13 MB per frame to do a few arithmetic operations per
// pixel. Fused, it reads the radiance once, reads the two fixed-pattern maps
// once, writes 8 bits per pixel, and touches nothing else — under 4 MB.
//
// Nothing about the ORDER changes, and that matters: §9.3's order is physical
// (atmosphere happens to the light, the black level is the pedestal the ADC
// sits on, shot noise is in photons, read noise is the electronics, the fixed
// pattern is the detector's own gain) and changing it would change the image.
// Fusing five sequential loops over the same index into one loop body is
// bit-exact by construction.
//
// ---------------------------------------------------------------------------
// WHY ONE GAUSSIAN DRAW INSTEAD OF TWO
// ---------------------------------------------------------------------------
// Above design §9.3's lambda = 30 cutoff, shot noise is N(lambda, lambda) and
// read noise is N(0, sigma^2). They are INDEPENDENT, so their sum is one normal
// whose variance is the sum of the variances:
//
//     v + sqrt( v/k + sigma^2 ) * Z        instead of
//     v + sqrt(v/k) * Z1 + sigma * Z2
//
// That is not an approximation — it is the same distribution, and it is how
// every sensor datasheet states total noise in the first place ("read and shot
// noise add in quadrature"). It halves the per-pixel Gaussian draws, which were
// most of the stage's cost.
//
// What it changes is which RNG stream is consumed, so the rule is stated
// explicitly here: the combined draw comes from Stream::ShotNoise whenever shot
// noise is enabled, and from Stream::ReadNoise when it is not. Turning read
// noise on or off therefore does not shift the shot-noise DRAW SEQUENCE — still
// exactly one per pixel — only its scale, which is the property the separate
// streams exist to provide. Turning shot noise off moves to the other stream.
//
// Below the cutoff, shot noise is genuinely discrete (Knuth's method) and
// cannot be folded into a Gaussian, so that branch still draws twice. With the
// specification's black level of 16 grey levels and 8 photons per level, the
// darkest possible pixel is lambda = 128, so this branch is unreachable in any
// configured scenario and exists for the fuzzer (CP 14.1), which is allowed to
// set the pedestal to zero.
// ---------------------------------------------------------------------------
void SensorChain::apply(std::span<float> radiance, std::span<uint8_t> out,
                        RngSet& rng) const {
    // INV-8: in a video mode the frame passes through untouched apart from the
    // quantisation it needs anyway.
    if (!enabled_) {
        quantise_u8(radiance, out);
        return;
    }

    const size_t n = std::min(radiance.size(), out.size());

    // --- everything the loop needs, hoisted -------------------------------
    const AtmosphereCoeffs atm   = atmosphere_coeffs(atmosphere_);
    const double           black = noise_.black_level;
    const double           k     = noise_.photons_per_level;
    const double           sigma = noise_.gaussian_sigma;

    const bool   shot      = noise_.poisson_enabled && k > 0.0;
    const bool   read      = sigma > 0.0;
    const double read_var  = read ? sigma * sigma : 0.0;
    const double inv_k     = shot ? 1.0 / k : 0.0;

    // ----------------------------------------------------------------------
    // ALIASING — why the generators are COPIED rather than used by reference
    // ----------------------------------------------------------------------
    // `out` is a span of uint8_t, and uint8_t is a character type, so the
    // compiler must assume a store through it can alias ANYTHING — including
    // the RNG state sitting inside `rng`. Written the obvious way, every
    // `out[i] = ...` forces the generator's 64-bit state to be reloaded from
    // memory on the next iteration, and the same for the hoisted constants.
    // The loop then runs at the speed of its stores rather than its arithmetic.
    //
    // Taking a local copy of each generator breaks that: a local whose address
    // never escapes cannot be aliased by anything, so the state lives in a
    // register for the whole pass and is written back once at the end. The
    // draw sequence is identical — this is a register-allocation fix, not a
    // numerical one.
    // ----------------------------------------------------------------------
    Pcg32& gmain_ref = shot ? rng[Stream::ShotNoise] : rng[Stream::ReadNoise];
    // Only used on the discrete-shot-noise branch, which the specification's
    // black level makes unreachable.
    Pcg32& gread_ref = rng[Stream::ReadNoise];
    Pcg32  gmain = gmain_ref;
    Pcg32  gread = gread_ref;

    // __restrict for the same reason: these three never overlap, and saying so
    // lets the loads be hoisted and the loop software-pipelined.
    const float* __restrict prnu = fixed_.prnu_map();
    const float* __restrict fpn  = fixed_.fpn_map();
    const float* __restrict rad  = radiance.data();
    uint8_t* __restrict      dst = out.data();
    const bool   fp   = prnu && fpn && fixed_.map_size() >= n;

    // Design §9.3's branch point between Knuth's exact Poisson and the normal
    // approximation. At lambda = 30 the skewness is 1/sqrt(30) = 0.18, well
    // below what 8-bit quantisation can express.
    constexpr float kPoissonNormalCutoff = 30.0f;

    // ----------------------------------------------------------------------
    // SINGLE PRECISION — why the whole loop is float and not double
    // ----------------------------------------------------------------------
    // The input is a float render, the output is eight bits, and the smallest
    // quantity that survives to the output is one grey level in 255. Float
    // carries seven significant digits, so the arithmetic here has five orders
    // of magnitude more precision than the thing it is computing.
    //
    // What double cost was conversions. The buffers are float, so every
    // double-precision statement was bracketed by a cvtss2sd going in and a
    // cvtsd2ss coming out, and the disassembly of the loop had SEVEN of them —
    // the radiance load, the normal deviate, both fixed-pattern maps and the
    // quantisation. At four to five cycles each that was a third of the loop.
    //
    // It also halves the vector width the AVX2 path can use later: eight floats
    // per register against four doubles.
    // ----------------------------------------------------------------------
    const float alpha_f = static_cast<float>(atm.alpha);
    const float beta_f  = static_cast<float>(atm.beta + black);   // one constant
    const float k_f     = static_cast<float>(k);
    const float inv_k_f = static_cast<float>(inv_k);
    const float sigma_f = static_cast<float>(sigma);
    const float rvar_f  = static_cast<float>(read_var);

    // ----------------------------------------------------------------------
    // CP 14.2's vector path, where it applies.
    //
    // It consumes whole vectors of pixels and leaves the generator exactly
    // where the scalar loop would have left it, so the two are interchangeable
    // at any pixel boundary and the remainder below finishes the frame. It
    // declines — returning 0 — on a machine without AVX2, and the loop below
    // then does the whole frame.
    //
    // It also declines when design §9.3's discrete-Poisson branch could fire,
    // because Knuth's method has no vector form. `lambda_floor` is the smallest
    // lambda any pixel of this frame can produce: the radiance is non-negative,
    // so the floor is the atmosphere's own offset. At the specification's black
    // level of 16 grey levels and 8 photons per level it is 128, four times the
    // cutoff, which is why the branch is unreachable in a configured scenario
    // and is checked rather than assumed.
    // ----------------------------------------------------------------------
    size_t i = 0;
    {
        const float lambda_floor = beta_f * k_f;
        const bool  discrete_possible =
            shot && !(lambda_floor >= kPoissonNormalCutoff);
        if (!discrete_possible) {
            DamageArgs da;
            da.rad   = rad;
            da.dst   = dst;
            da.prnu  = prnu;
            da.fpn   = fpn;
            da.n     = n;
            da.alpha = alpha_f;
            da.beta  = beta_f;
            da.k     = k_f;
            da.inv_k = inv_k_f;
            da.sigma = sigma_f;
            da.rvar  = rvar_f;
            da.shot  = shot;
            da.read  = read;
            da.fp    = fp;
            i = damage_chain_simd(da, gmain);
        }
    }

    for (; i < n; ++i) {
        // --- 1. Atmosphere (spec row 24), then the black level -------------
        // The black level is added HERE — after the atmosphere, before any
        // noise — because that is where a real sensor applies it: the pedestal
        // exists so that the noise distribution sits above zero and is not
        // clipped by the ADC. Adding it later would be too late; the clipping
        // would already have happened. It is folded into beta because both are
        // constants and the sum is exact in float.
        float v = alpha_f * rad[i] + beta_f;

        // --- 2/3. Shot noise (row 21) and read noise (rows 21-22) ----------
        if (shot) {
            const float lambda = v * k_f;
            if (lambda >= kPoissonNormalCutoff) {
                // Gaussian regime: one draw for both, variances added.
                const float var = v * inv_k_f + rvar_f;
                v += std::sqrt(var) * fast_normal(gmain.next_u32());
            } else if (lambda > 0.0f) {
                // Discrete regime: Knuth, in photons, then back to grey levels.
                // Kept in double because poisson() is, and because this branch
                // is unreachable at the specification's black level anyway.
                v = static_cast<float>(poisson(static_cast<double>(lambda), gmain) * inv_k);
                if (read) v += sigma_f * fast_normal(gread.next_u32());
            } else if (read) {
                // No photons, no shot noise — but the electronics still hums.
                v += sigma_f * fast_normal(gread.next_u32());
            }
        } else if (read) {
            v += sigma_f * fast_normal(gmain.next_u32());
        }

        // --- 4. Fixed pattern ----------------------------------------------
        if (fp) {
            v = v * prnu[i] + fpn[i];
        }

        // --- 7a. Clip and quantise -----------------------------------------
        // Done before salt-and-pepper and defects because those write 8-bit
        // values directly: an impulse is a stuck ADC code, not a radiance.
        // Identical to core/image.hpp's quantise_u8, inlined so the fused loop
        // does not have to write the float back out just to read it again.
        const float f = v;
        if (!(f > 0.0f))       dst[i] = 0;
        else if (f >= 254.5f)  dst[i] = 255;
        else                   dst[i] = static_cast<uint8_t>(f + 0.5f);
        // NOTE: the radiance buffer is deliberately NOT written back. The old
        // five-pass chain degraded it in place, which cost a 1.2 MB store per
        // frame and left SyntheticSource::radiance() returning the degraded
        // image — the opposite of what its own documentation promises ("the
        // float render, BEFORE quantisation ... the centroid-accuracy harness
        // needs the undegraded image"). Nothing read it, so nothing broke; it
        // now means what it says.
    }

    // The generators' final state goes back where the rest of the simulation
    // will look for it. Skipping this would silently reset the noise sequence
    // every frame, which INV-3 would not catch — it is deterministic, just
    // wrong.
    gmain_ref = gmain;
    gread_ref = gread;

    // --- 5. Salt and pepper (spec row 21) ----------------------------------
    // Sparse by construction: geometric skip sampling visits only the pixels it
    // corrupts, so this is ~10% of n regardless of how the loop above is
    // written, and there is nothing to fuse.
    salt_pepper(out, noise_.salt_pepper_p, rng[Stream::SaltPepper]);

    // --- 6. Hot and dead pixels --------------------------------------------
    // After salt-and-pepper so a defect is never masked by a random impulse. A
    // stuck pixel stays stuck; that is what makes it a defect rather than noise,
    // and it is what a tracker must learn to ignore in the same places frame
    // after frame.
    if (defects_enabled_) fixed_.apply_defects(out);
}

}  // namespace sat
