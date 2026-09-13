// tests/core/test_time.cpp — CP 0.3, the clock half.
//
// "Clock rejects truth_hz % camera_hz != 0"

#include <doctest/doctest.h>

#include "core/time.hpp"

using namespace sat;

TEST_CASE("the default 300/30/30 rate structure is accepted") {
    auto r = Clock::make(300, 30, 30);
    REQUIRE(r.has_value());
    const Clock c = *r;
    CHECK(c.camera_divisor() == 10);      // design §6.2: 10 truth ticks per frame
    CHECK(c.control_divisor() == 10);
    CHECK(c.truth_dt() == doctest::Approx(1.0 / 300.0));
    CHECK(c.camera_dt() == doctest::Approx(1.0 / 30.0));
}

TEST_CASE("a truth rate that does not divide cleanly is rejected") {
    // 300 Hz truth with a 45 Hz camera would put frames between truth ticks,
    // forcing interpolation and breaking INV-3.
    auto r = Clock::make(300, 45, 30);
    CHECK_FALSE(r.has_value());
    CHECK(r.error().find("exact multiple") != std::string::npos);
    CHECK(r.error().find("camera_hz") != std::string::npos);
}

TEST_CASE("a control rate that does not divide cleanly is rejected") {
    auto r = Clock::make(300, 30, 40);
    CHECK_FALSE(r.has_value());
    CHECK(r.error().find("control_hz") != std::string::npos);
}

TEST_CASE("rates below the specification minima are rejected, citing the row") {
    SUBCASE("camera below 30 Hz is specification row 5") {
        auto r = Clock::make(300, 25, 25);
        CHECK_FALSE(r.has_value());
        CHECK(r.error().find("row 5") != std::string::npos);
    }
    SUBCASE("control below 20 Hz is specification row 15") {
        auto r = Clock::make(300, 30, 10);
        CHECK_FALSE(r.has_value());
        CHECK(r.error().find("row 15") != std::string::npos);
    }
    SUBCASE("a non-positive truth rate is rejected") {
        CHECK_FALSE(Clock::make(0, 30, 30).has_value());
    }
}

TEST_CASE("seconds() is derived from the integer tick, never accumulated") {
    auto c = *Clock::make(300, 30, 30);

    // The acceptance criterion for CP 2.1. Accumulating 1/300 three hundred
    // times gives 0.9999999999999062 on IEEE doubles; exact division gives
    // exactly 1.0. Over a 120 s run the accumulated version drifts by enough
    // to move a fast target by a measurable fraction of a pixel.
    for (int i = 0; i < 300; ++i) c.tick();
    CHECK(c.seconds() == 1.0);              // exact equality on purpose

    for (int i = 0; i < 300 * 119; ++i) c.tick();
    CHECK(c.seconds() == 120.0);            // still exact after two minutes
    CHECK(c.tick_index() == 36000);
}

TEST_CASE("camera and control ticks land on the expected truth ticks") {
    auto c = *Clock::make(300, 30, 30);

    int frames = 0;
    // Tick 0 is a frame: the run starts by looking at the world.
    CHECK(c.is_camera_tick());
    CHECK(c.is_control_tick());

    for (int i = 0; i < 300; ++i) {
        if (c.is_camera_tick()) ++frames;
        c.tick();
    }
    CHECK(frames == 30);                    // exactly one second of 30 Hz frames
    CHECK(c.frame_index() == 30);
}

TEST_CASE("mixed rates: 600 Hz truth, 60 Hz camera, 20 Hz control") {
    auto r = Clock::make(600, 60, 20);
    REQUIRE(r.has_value());
    Clock c = *r;
    CHECK(c.camera_divisor() == 10);
    CHECK(c.control_divisor() == 30);

    int cam = 0, ctl = 0;
    for (int i = 0; i < 600; ++i) {
        if (c.is_camera_tick())  ++cam;
        if (c.is_control_tick()) ++ctl;
        c.tick();
    }
    CHECK(cam == 60);
    CHECK(ctl == 20);
}

TEST_CASE("ticks_for() rounds to whole ticks so runs are identical in length") {
    auto c = *Clock::make(300, 30, 30);
    CHECK(c.ticks_for(120.0) == 36000);
    CHECK(c.ticks_for(0.0) == 0);
    // A duration that is not a whole number of ticks rounds to the nearest.
    CHECK(c.ticks_for(1.0 / 600.0) == 1);   // half a tick rounds up
}

TEST_CASE("reset() rewinds time without losing the configured rates") {
    auto c = *Clock::make(300, 30, 30);
    for (int i = 0; i < 1234; ++i) c.tick();
    c.reset();
    CHECK(c.tick_index() == 0);
    CHECK(c.seconds() == 0.0);
    CHECK(c.camera_divisor() == 10);        // configuration survives
}
