// tests/core/test_triple_buffer.cpp — CP 0.4 / CP 2.3 groundwork.
//
// The property that matters (CP 2.3): "Artificially slowing the display to
// 5 FPS does not slow the simulation." A triple buffer delivers that by never
// blocking the producer -- it drops intermediate snapshots instead.

#include <doctest/doctest.h>

#include "core/triple_buffer.hpp"

#include <atomic>
#include <thread>

using namespace sat;

namespace {
// A stand-in for SimSnapshot: a value plus a checksum, so a torn read (the
// consumer seeing half of one snapshot and half of another) is detectable.
struct Snap {
    int      value    = 0;
    int      echo     = 0;
    uint64_t checksum = 0;

    void set(int v) {
        value = v;
        echo  = v;
        checksum = static_cast<uint64_t>(v) * 2654435761u;
    }
    [[nodiscard]] bool consistent() const {
        return echo == value && checksum == static_cast<uint64_t>(value) * 2654435761u;
    }
};
}  // namespace

TEST_CASE("nothing is available before the first publish") {
    TripleBuffer<Snap> tb;
    CHECK_FALSE(tb.acquire());
    CHECK(tb.published_count() == 0);
}

TEST_CASE("publish then acquire delivers the snapshot") {
    TripleBuffer<Snap> tb;
    tb.write_slot().set(42);
    tb.publish();

    REQUIRE(tb.acquire());
    CHECK(tb.read_slot().value == 42);
    CHECK(tb.read_slot().consistent());

    // A second acquire with nothing new published must report "nothing new"
    // so the GUI keeps drawing what it already has.
    CHECK_FALSE(tb.acquire());
}

TEST_CASE("the consumer always gets the NEWEST snapshot, not a queue") {
    // This is the behaviour that keeps the display from falling behind. A
    // queue would build a backlog; a triple buffer drops.
    TripleBuffer<Snap> tb;
    for (int i = 1; i <= 100; ++i) {
        tb.write_slot().set(i);
        tb.publish();
    }
    REQUIRE(tb.acquire());
    CHECK(tb.read_slot().value == 100);
    CHECK(tb.published_count() == 100);
    CHECK(tb.consumed_count() == 1);
}

TEST_CASE("the write slot is never the slot the consumer is reading") {
    // The whole safety argument rests on this: three slots mean the producer
    // always has a free one, even while the consumer holds one and another is
    // sitting ready.
    TripleBuffer<Snap> tb;
    tb.write_slot().set(1);
    tb.publish();
    REQUIRE(tb.acquire());

    const Snap* held = &tb.read_slot();
    for (int i = 2; i < 50; ++i) {
        CHECK(&tb.write_slot() != held);      // producer never touches it
        tb.write_slot().set(i);
        tb.publish();
    }
    CHECK(held->value == 1);                  // the consumer's copy is intact
}

TEST_CASE("a fast producer and a slow consumer never tear or block") {
    // The concurrency test. The producer runs flat out; the consumer deliberately
    // dawdles. Every snapshot the consumer sees must be internally consistent
    // (no tearing) and monotonically newer (no going backwards).
    TripleBuffer<Snap> tb;
    std::atomic<bool> stop{false};
    std::atomic<int>  produced{0};

    std::thread producer([&] {
        for (int i = 1; i <= 200000 && !stop.load(std::memory_order_relaxed); ++i) {
            tb.write_slot().set(i);
            tb.publish();
            produced.store(i, std::memory_order_relaxed);
        }
        stop.store(true, std::memory_order_relaxed);
    });

    int  last = 0;
    int  seen = 0;
    bool torn = false;
    bool went_backwards = false;

    // ONE acquire per iteration.
    //
    // An earlier version called acquire() in the loop CONDITION and again in
    // the body. The condition's call consumed a snapshot that the body's call
    // then could not see, so a consumer that was never scheduled during
    // production would finish with seen == 0 and fail. That is precisely what
    // happened in CI: intermittent, Release-only, never in Debug — because an
    // optimised producer can run all 200000 iterations before a consumer on a
    // loaded two-core runner gets scheduled at all.
    //
    // The loop now reads `stop` BEFORE acquiring, so the final iteration still
    // drains whatever was published last.
    for (;;) {
        const bool finished = stop.load(std::memory_order_relaxed);
        if (tb.acquire()) {
            const Snap& s = tb.read_slot();
            if (!s.consistent())  torn = true;
            if (s.value < last)   went_backwards = true;
            last = s.value;
            ++seen;
        } else if (finished) {
            break;
        }
        // Simulate a slow display: give up the rest of our slice.
        std::this_thread::yield();
    }
    producer.join();

    CHECK_FALSE(torn);
    CHECK_FALSE(went_backwards);

    // At least one snapshot must arrive. This is now guaranteed rather than
    // hoped for: the producer publishes 200000 times, and the drain above runs
    // until acquire() reports nothing new, so even a consumer that never ran
    // during production picks up the last one.
    INFO("produced " << produced.load() << ", consumer saw " << seen);
    CHECK(seen > 0);
    CHECK(seen <= produced.load());

    // Dropping frames is the intended behaviour, but it is NOT asserted here:
    // whether the consumer drops anything depends on the scheduler, and a test
    // that requires a particular interleaving is a test that fails on someone
    // else's machine. The property that matters -- newest-wins, never torn,
    // never backwards -- is asserted above and holds regardless of timing.
}

TEST_CASE("constructing from a prototype pre-sizes all three slots") {
    // SimSnapshot owns a downsampled preview image. Sizing it once at
    // construction is what keeps publish() allocation-free (INV-4).
    Snap proto;
    proto.set(7);
    TripleBuffer<Snap> tb(proto);
    tb.publish();
    REQUIRE(tb.acquire());
    CHECK(tb.read_slot().value == 7);
}
