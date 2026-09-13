// degrade/sensor.hpp — the full damage chain, in order.
//
// This is design §9.3 assembled: atmosphere, shot noise, read noise, fixed
// pattern, salt and pepper, defects, clip and quantise. See degrade/noise.hpp
// for why the order is what it is.
//
// ---------------------------------------------------------------------------
// INV-8 LIVES HERE
// ---------------------------------------------------------------------------
// "No damage is added in video modes. When input.mode != synthetic, the noise
// and atmosphere generators are forcibly disabled. The degradation is already
// in the supplied footage; adding more corrupts the benchmark."
//
// The chain takes an `enabled` flag rather than checking the mode itself, so
// the decision is made once at construction from Scenario::damage_enabled() and
// cannot be forgotten at a call site.

#pragma once

#include "core/rng.hpp"
#include "degrade/noise.hpp"
#include "scenario/scenario.hpp"

#include <cstdint>
#include <span>

namespace sat {

class SensorChain {
public:
    /// Configure from a scenario. Draws the fixed-pattern maps once (design
    /// §6.1 step A4/A5); after this, applying the chain allocates nothing.
    void build(const Scenario& sc, int width, int height, RngSet& rng);

    /// Run the whole chain: float radiance in, 8-bit frame out.
    ///
    /// `radiance` is modified in place (it is scratch owned by the source), and
    /// `out` receives the quantised result.
    void apply(std::span<float> radiance, std::span<uint8_t> out, RngSet& rng) const;

    /// Change the weather mid-run, for design §7.4's `set_atmosphere` event.
    void set_atmosphere(Atmosphere a) noexcept { atmosphere_ = a; }
    [[nodiscard]] Atmosphere atmosphere() const noexcept { return atmosphere_; }

    /// INV-8's switch. False in every video mode, and false until build() is
    /// called at all.
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    /// Turn the chain on or off at runtime. The live demo (CP 15.2) dials
    /// damage up and down on stage, and the ablation table needs a clean arm.
    void set_enabled(bool on) noexcept { enabled_ = on; }

    [[nodiscard]] const NoiseParams& noise() const noexcept { return noise_; }
    [[nodiscard]] NoiseParams&       noise()       noexcept { return noise_; }
    [[nodiscard]] const FixedPattern& fixed_pattern() const noexcept { return fixed_; }

private:
    // Defaults to DISABLED, so a chain nobody configured is a pass-through
    // rather than one silently applying spec-maximum noise. build() decides
    // from Scenario::damage_enabled(). An earlier version defaulted to true and
    // made every hand-built test scene arrive at the detector under 20-sigma
    // read noise and 10% salt-and-pepper.
    bool         enabled_    = false;
    Atmosphere   atmosphere_ = Atmosphere::Clear;
    NoiseParams  noise_{};
    FixedPattern fixed_{};
};

}  // namespace sat
