// tests/core/test_ring.cpp — CP 0.4.
//
// "Ring passes wraparound tests"
//
// The delay-line behaviour is the one that matters: design §10.3 reads
// `at_back(d)` to model the gimbal's transport lag, and an off-by-one there
// shifts every command by 33 ms.

#include <doctest/doctest.h>

#include "core/ring.hpp"

#include <vector>

using namespace sat;

TEST_CASE("a fresh ring is empty") {
    Ring<int, 8> r;
    CHECK(r.empty());
    CHECK(r.size() == 0);
    CHECK_FALSE(r.full());
    CHECK(Ring<int, 8>::capacity() == 8);
}

TEST_CASE("push fills up to capacity then overwrites the oldest") {
    Ring<int, 4> r;
    for (int i = 0; i < 4; ++i) r.push(i);
    CHECK(r.full());
    CHECK(r.size() == 4);
    CHECK(r.newest() == 3);
    CHECK(r.oldest() == 0);

    r.push(4);                      // evicts 0
    CHECK(r.size() == 4);
    CHECK(r.newest() == 4);
    CHECK(r.oldest() == 1);
}

TEST_CASE("at_back indexes backwards from the most recent push") {
    Ring<int, 8> r;
    for (int i = 0; i < 8; ++i) r.push(i * 10);
    CHECK(r.at_back(0) == 70);      // most recent
    CHECK(r.at_back(1) == 60);
    CHECK(r.at_back(7) == 0);       // oldest still held
}

TEST_CASE("at_back survives many wraparounds") {
    // The gimbal pushes a command every truth tick for the whole run, so the
    // ring wraps thousands of times. The delayed read must stay correct
    // throughout.
    Ring<int, 16> r;
    for (int i = 0; i < 10000; ++i) {
        r.push(i);
        CHECK(r.at_back(0) == i);
        if (i >= 3)  CHECK(r.at_back(3) == i - 3);
        if (i >= 15) CHECK(r.at_back(15) == i - 15);
    }
}

TEST_CASE("at_back clamps rather than reading uninitialised memory") {
    // At startup the delay line has fewer entries than the requested lag. The
    // physically sensible answer is "the oldest command we have", not garbage.
    Ring<int, 8> r;
    r.push(5);
    CHECK(r.at_back(0) == 5);
    CHECK(r.at_back(3) == 5);       // clamped to the only element
    CHECK(r.at_back(100) == 5);

    // An empty ring must also be safe to read.
    Ring<int, 8> e;
    CHECK(e.at_back(0) == 0);
}

TEST_CASE("fill primes the whole ring") {
    // This is what the gimbal does at startup: before any command arrives, the
    // motor sees a commanded rate of zero, not whatever was in memory.
    Ring<double, 16> r;
    r.fill(0.0);
    CHECK(r.full());
    CHECK(r.size() == 16);
    for (size_t d = 0; d < 16; ++d) CHECK(r.at_back(d) == 0.0);

    r.push(1.5);
    CHECK(r.at_back(0) == 1.5);
    CHECK(r.at_back(1) == 0.0);
}

TEST_CASE("clear empties without disturbing capacity") {
    Ring<int, 4> r;
    for (int i = 0; i < 10; ++i) r.push(i);
    r.clear();
    CHECK(r.empty());
    r.push(42);
    CHECK(r.newest() == 42);
    CHECK(r.size() == 1);
}

TEST_CASE("for_each visits oldest to newest") {
    Ring<int, 4> r;
    for (int i = 0; i < 6; ++i) r.push(i);   // holds 2,3,4,5
    std::vector<int> seen;
    r.for_each([&](int v) { seen.push_back(v); });
    REQUIRE(seen.size() == 4);
    CHECK(seen[0] == 2);
    CHECK(seen[1] == 3);
    CHECK(seen[2] == 4);
    CHECK(seen[3] == 5);
}

TEST_CASE("a 64-entry ring covers the worst-case transport delay") {
    // Design §10.3 sizes the delay line at 64. At the maximum sensible truth
    // rate the lag must still fit: 0.010 s latency x 600 Hz = 6 entries, so 64
    // leaves an order of magnitude of headroom.
    constexpr double kLatencyS = 0.010;
    constexpr double kTruthHz  = 600.0;
    const int needed = static_cast<int>(kLatencyS * kTruthHz + 0.5);
    CHECK(needed < static_cast<int>(Ring<double, 64>::capacity()));
}
