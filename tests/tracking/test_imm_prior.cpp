// tests/tracking/test_imm_prior.cpp — ImmFilter::set_regime_prior.

#include <doctest/doctest.h>

#include "tracking/imm.hpp"
#include "tracking/measurement.hpp"

using namespace sat;

namespace {

Measurement meas(double x, double y, double sigma = 100.0) {
    Measurement m;
    m.angle = Angle2{x, y};
    m.sigma_urad = sigma;
    return m;
}

}  // namespace

TEST_CASE("set_regime_prior(Figure8) raises CT versus a control that did not") {
    constexpr double dt = 1.0 / 30.0;
    constexpr double vx = 87266.46;
    ImmParams p = ImmParams::from_max_accel(2.0e4, dt);

    ImmFilter boosted, control;
    boosted.init(meas(0.0, 0.0), p);
    control.init(meas(0.0, 0.0), p);

    for (int i = 1; i <= 40; ++i) {
        const double x = vx * static_cast<double>(i) * dt;
        boosted.predict(dt);
        control.predict(dt);
        boosted.update(Angle2{x, 0.0}, 100.0);
        control.update(Angle2{x, 0.0}, 100.0);
    }

    boosted.set_regime_prior(MotionRegime::Figure8, 1.0f);
    boosted.predict(dt);
    control.predict(dt);

    CHECK(boosted.mode_prob(ImmMode::CT) > control.mode_prob(ImmMode::CT));
}
