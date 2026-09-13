#include "degrade/sensor.hpp"

#include "camera/splat.hpp"   // quantise_u8

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

void SensorChain::apply(std::span<float> radiance, std::span<uint8_t> out,
                        RngSet& rng) const {
    // INV-8: in a video mode the frame passes through untouched apart from the
    // quantisation it needs anyway.
    if (!enabled_) {
        quantise_u8(radiance, out);
        return;
    }

    const size_t n = std::min(radiance.size(), out.size());

    // --- 1. Atmosphere (spec row 24) ---------------------------------------
    // An affine transform, applied before anything else because it describes
    // what happens to the light on its way to the detector.
    const AtmosphereCoeffs atm = atmosphere_coeffs(atmosphere_);
    if (atm.alpha != 1.0 || atm.beta != 0.0) {
        for (size_t i = 0; i < n; ++i) {
            radiance[i] = static_cast<float>(atm.alpha * radiance[i] + atm.beta);
        }
    }

    // --- 2. Shot noise (spec row 21) ---------------------------------------
    // Poisson in PHOTONS, then back to grey levels. Doing it in grey levels
    // directly would make the noise depend on the arbitrary choice of scale.
    if (noise_.poisson_enabled && noise_.photons_per_level > 0.0) {
        Pcg32& g = rng[Stream::ShotNoise];
        const double k = noise_.photons_per_level;
        for (size_t i = 0; i < n; ++i) {
            const double lambda = static_cast<double>(radiance[i]) * k;
            if (lambda <= 0.0) continue;   // no photons, no shot noise
            radiance[i] = static_cast<float>(poisson(lambda, g) / k);
        }
    }

    // --- 3. Read noise (spec rows 21-22) -----------------------------------
    // Additive and signal-independent: it is the electronics, not the light.
    if (noise_.gaussian_sigma > 0.0) {
        Pcg32& g = rng[Stream::ReadNoise];
        for (size_t i = 0; i < n; ++i) {
            radiance[i] += static_cast<float>(noise_.gaussian_sigma * g.next_normal());
        }
    }

    // --- 4. Fixed pattern --------------------------------------------------
    fixed_.apply_gain_offset(radiance);

    // --- 7a. Clip and quantise ---------------------------------------------
    // Done before salt-and-pepper and defects because those write 8-bit values
    // directly: an impulse is a stuck ADC code, not a radiance.
    quantise_u8(radiance, out);

    // --- 5. Salt and pepper (spec row 21) ----------------------------------
    salt_pepper(out, noise_.salt_pepper_p, rng[Stream::SaltPepper]);

    // --- 6. Hot and dead pixels --------------------------------------------
    // After salt-and-pepper so a defect is never masked by a random impulse. A
    // stuck pixel stays stuck; that is what makes it a defect rather than noise,
    // and it is what a tracker must learn to ignore in the same places frame
    // after frame.
    fixed_.apply_defects(out);
}

}  // namespace sat
