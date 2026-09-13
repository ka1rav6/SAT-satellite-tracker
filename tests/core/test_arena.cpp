// tests/core/test_arena.cpp — CP 0.4.
//
// "Arena survives 10k alloc/release cycles without growing"

#include <doctest/doctest.h>

#include "core/arena.hpp"

#include <cstdint>

using namespace sat;

TEST_CASE("a fresh arena hands out the space it was given") {
    Arena a(1024);
    CHECK(a.capacity() == 1024);
    CHECK(a.used() == 0);

    auto s = a.alloc<int32_t>(16);
    REQUIRE(s.size() == 16);
    CHECK(a.used() >= 64);
    CHECK(a.exhaustions() == 0);
}

TEST_CASE("10k alloc/reset cycles do not grow the arena") {
    // The INV-4 acceptance test. If reset() leaked, `used()` would creep up and
    // the arena would eventually start reporting exhaustions.
    Arena a(64 * 1024);
    for (int i = 0; i < 10000; ++i) {
        auto img  = a.alloc<uint8_t>(4096);
        auto sums = a.alloc<int64_t>(512);
        auto runs = a.alloc<int16_t>(256);
        REQUIRE_FALSE(img.empty());
        REQUIRE_FALSE(sums.empty());
        REQUIRE_FALSE(runs.empty());
        // Touch the memory so a compiler cannot elide the allocations.
        img[0] = static_cast<uint8_t>(i);
        sums[0] = i;
        runs[0] = static_cast<int16_t>(i);
        a.reset();
    }
    CHECK(a.used() == 0);
    CHECK(a.capacity() == 64 * 1024);
    CHECK(a.exhaustions() == 0);
    // High water reflects one cycle's peak, not 10000 of them.
    CHECK(a.high_water() < 64 * 1024);
}

TEST_CASE("allocations are aligned for AVX2") {
    // The SIMD kernels in design §9.4 want 32-byte alignment; getting it for
    // free from the arena means the kernels never need an unaligned prologue.
    Arena a(4096);
    for (int i = 0; i < 20; ++i) {
        auto s = a.alloc<uint8_t>(static_cast<size_t>(1 + i * 7));  // awkward sizes
        REQUIRE_FALSE(s.empty());
        CHECK(reinterpret_cast<uintptr_t>(s.data()) % 32 == 0);
    }
}

TEST_CASE("exhaustion returns an empty span rather than throwing") {
    // Deliberate design choice: a mis-sized arena must not take a live demo
    // down. The caller checks for empty, and the metrics layer reports the
    // exhaustion count.
    Arena a(256);
    auto ok = a.alloc<uint8_t>(128);
    CHECK_FALSE(ok.empty());

    auto too_big = a.alloc<uint8_t>(1024);
    CHECK(too_big.empty());
    CHECK(a.exhaustions() == 1);

    // The arena is still usable for something that fits.
    auto still_ok = a.alloc<uint8_t>(32);
    CHECK_FALSE(still_ok.empty());
}

TEST_CASE("alloc_zeroed really zeroes") {
    Arena a(1024);
    auto dirty = a.alloc<uint32_t>(64);
    for (auto& v : dirty) v = 0xDEADBEEFu;
    a.reset();

    auto clean = a.alloc_zeroed<uint32_t>(64);
    REQUIRE(clean.size() == 64);
    for (auto v : clean) CHECK(v == 0u);
}

TEST_CASE("zero-count allocation is a no-op") {
    Arena a(256);
    auto s = a.alloc<int>(0);
    CHECK(s.empty());
    CHECK(a.used() == 0);
}

TEST_CASE("ScopedArena restores the bump pointer on scope exit") {
    Arena a(4096);
    auto outer = a.alloc<uint8_t>(64);
    REQUIRE_FALSE(outer.empty());
    const size_t after_outer = a.used();

    {
        ScopedArena scope(a);
        auto inner = scope.get().alloc<uint8_t>(1024);
        REQUIRE_FALSE(inner.empty());
        CHECK(a.used() > after_outer);
    }
    CHECK(a.used() == after_outer);

    // The reclaimed space is genuinely reusable.
    auto reused = a.alloc<uint8_t>(1024);
    CHECK_FALSE(reused.empty());
}

TEST_CASE("ArenaSet reserves the four arenas from design 6.1 A4") {
    ArenaSet set;
    set.reserve_defaults();
    CHECK(set.persistent.capacity() == ArenaSet::kDefaultPersistentBytes);
    CHECK(set.frame.capacity()      == ArenaSet::kDefaultFrameBytes);
    CHECK(set.video.capacity()      == ArenaSet::kDefaultVideoBytes);
    CHECK(set.report.capacity()     == ArenaSet::kDefaultReportBytes);

    // A 640x480 frame needs: filtered u8 (307k), top-hat i16 (614k), two SATs
    // (641*481*8 = 2.5 MB and the same again for sum-of-squares). 8 MB has
    // comfortable headroom for that plus the run list and candidates.
    auto sat_sum = set.frame.alloc<int64_t>(641 * 481);
    auto sat_sq  = set.frame.alloc<uint64_t>(641 * 481);
    auto tophat  = set.frame.alloc<int16_t>(640 * 480);
    auto filt    = set.frame.alloc<uint8_t>(640 * 480);
    CHECK_FALSE(sat_sum.empty());
    CHECK_FALSE(sat_sq.empty());
    CHECK_FALSE(tophat.empty());
    CHECK_FALSE(filt.empty());
    CHECK(set.frame.exhaustions() == 0);
}

TEST_CASE("FrameScope toggles the allocation-trap flag") {
    CHECK_FALSE(g_in_frame);
    {
        FrameScope fs;
        CHECK(g_in_frame);
    }
    CHECK_FALSE(g_in_frame);
}
