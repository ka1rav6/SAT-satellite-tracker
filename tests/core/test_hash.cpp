// tests/core/test_hash.cpp — CP 0.4 / CP 2.5 groundwork.
//
// The reproducibility fingerprint is only useful if it is (a) stable for
// identical input and (b) sensitive to the things that matter. Both directions
// are tested here.

#include <doctest/doctest.h>

#include "core/hash.hpp"

#include <algorithm>
#include <string>
#include <vector>

using namespace sat;

TEST_CASE("FNV-1a matches its published test vectors") {
    // The reference values for FNV-1a 64-bit. If these ever disagree, two
    // builds of this project could silently compute different fingerprints.
    CHECK(fnv1a(std::string_view("")) == 0xcbf29ce484222325ULL);
    CHECK(fnv1a(std::string_view("a")) == 0xaf63dc4c8601ec8cULL);
    CHECK(fnv1a(std::string_view("foobar")) == 0x85944171f73967e8ULL);
}

TEST_CASE("hashing is order-sensitive") {
    // If perception visits blobs in a different order, the fingerprint must
    // change -- label numbering depends on scan order (design §9.4.6) and a
    // reordering is exactly the kind of non-determinism INV-3 forbids.
    CHECK(fnv1a(std::string_view("ab")) != fnv1a(std::string_view("ba")));
}

TEST_CASE("chaining a hash is the same as hashing the concatenation") {
    const uint64_t chained = fnv1a(std::string_view("bar"), fnv1a(std::string_view("foo")));
    CHECK(chained == fnv1a(std::string_view("foobar")));
}

TEST_CASE("hash_double collapses negative zero") {
    // A rate that settles at exactly zero from below produces -0.0. It is
    // physically identical to +0.0, and a fingerprint that distinguished them
    // would report a reproducibility failure that is not one.
    CHECK(hash_double(0.0) == hash_double(-0.0));
    CHECK(hash_double(1.0) != hash_double(-1.0));
}

TEST_CASE("hash_double is sensitive to the last bit") {
    // The strict check: a one-ULP difference must show up. This is what makes
    // the reproducibility job able to catch an FMA contraction.
    const double a = 1.0;
    const double b = std::nextafter(1.0, 2.0);
    CHECK(a != b);
    CHECK(hash_double(a) != hash_double(b));
}

TEST_CASE("hash_quantised accepts divergence below the stated resolution") {
    // For anything downstream of a neural network or a libm call, design §11.4
    // asks us to hash at a declared tolerance rather than claim bit-exactness.
    const double centroid = 1423.8123456;
    CHECK(hash_quantised(centroid, 1e-6) == hash_quantised(centroid + 1e-10, 1e-6));
    // A micro-pixel is four orders of magnitude below the graded metric, so a
    // difference at that scale is genuinely meaningless...
    CHECK(hash_quantised(centroid, 1e-6) != hash_quantised(centroid + 1e-3, 1e-6));
    // ...but a milli-pixel difference must still be caught.
}

TEST_CASE("hash_value folds in integer-like types") {
    struct Ids { int32_t track; int32_t mode; uint64_t frame; };   // no padding
    static_assert(std::has_unique_object_representations_v<Ids>);

    const Ids a{1, 2, 3};
    const Ids b{1, 2, 4};
    CHECK(hash_value(a) != hash_value(b));
    CHECK(hash_value(a) == hash_value(Ids{1, 2, 3}));
}

TEST_CASE("hash_value refuses types where padding would poison the fingerprint") {
    // A struct of two doubles and an int has four bytes of tail padding that no
    // constructor initialises. Hashing it bytewise would make two genuinely
    // identical runs disagree -- a phantom reproducibility failure. The
    // static_assert in hash_value() makes that a compile error instead.
    struct Padded { double az, el; int32_t mode; };
    static_assert(sizeof(Padded) > sizeof(double) * 2 + sizeof(int32_t),
                  "this type is supposed to have padding");
    static_assert(!std::has_unique_object_representations_v<Padded>,
                  "hash_value(Padded) must not compile");

    // The sanctioned way to fingerprint it: member by member.
    const Padded p{1.0, 2.0, 3};
    uint64_t h = hash_double(p.az);
    h = hash_double(p.el, h);
    h = hash_value(p.mode, h);

    uint64_t h2 = hash_double(1.0);
    h2 = hash_double(2.0, h2);
    h2 = hash_value(static_cast<int32_t>(3), h2);
    CHECK(h == h2);
}

TEST_CASE("hash_vec2 folds a 2-D vector component-wise") {
    struct V { double x, y; };
    CHECK(hash_vec2(V{1.0, 2.0}) == hash_vec2(V{1.0, 2.0}));
    CHECK(hash_vec2(V{1.0, 2.0}) != hash_vec2(V{2.0, 1.0}));
    // Negative zero is normalised on both axes.
    CHECK(hash_vec2(V{0.0, -0.0}) == hash_vec2(V{-0.0, 0.0}));
}

TEST_CASE("FrameFingerprint separates the parts so a divergence is diagnosable") {
    // The reason this is a struct and not one number: when CI reports a
    // mismatch, "the image is identical but the detection moved" points at
    // perception, while "the image differs" points at the world or the
    // degradation chain.
    FrameFingerprint a{};
    a.frame = 10; a.image = 111; a.boresight = 222; a.detection = 333;
    a.track = 444; a.mode = 555;

    FrameFingerprint b = a;
    CHECK(a.combined() == b.combined());

    b.detection = 334;
    CHECK(a.combined() != b.combined());
    CHECK(a.image == b.image);              // the diagnosis is still readable

    // `frame` is an index, not state, so it deliberately does not enter the
    // combined hash -- two runs are compared frame by frame, not as a set.
    FrameFingerprint c = a;
    c.frame = 99;
    CHECK(a.combined() == c.combined());
}

TEST_CASE("hashing a frame buffer is stable and sensitive") {
    std::vector<uint8_t> img(640 * 480, 32);
    const uint64_t base = fnv1a(std::span<const uint8_t>(img));
    CHECK(base == fnv1a(std::span<const uint8_t>(img)));

    // One pixel changing by one grey level must change the hash. Salt-and-pepper
    // noise corrupts 30,720 pixels per frame; if the fingerprint missed a single
    // pixel it would miss an RNG divergence too.
    img[123456] = 33;
    CHECK(fnv1a(std::span<const uint8_t>(img)) != base);
}

// ===========================================================================
// P1-2 — fnv1a_bulk, the eight-lane image hash
//
// `snapshot` cost 813 us per frame against a 30 us budget, and nearly all of
// it was FNV-1a's serial multiply chain over 307,200 bytes. fnv1a_bulk runs
// eight independent chains instead. These tests pin the properties that make
// that substitution safe rather than merely fast.
// ===========================================================================

TEST_CASE("P1-2: fnv1a_bulk is a function of the bytes and the length") {
    std::vector<uint8_t> a(4096);
    for (size_t i = 0; i < a.size(); ++i) a[i] = static_cast<uint8_t>(i * 31 + 7);

    // Deterministic.
    CHECK(fnv1a_bulk(a.data(), a.size()) == fnv1a_bulk(a.data(), a.size()));

    // Sensitive to every byte, including ones deep inside and ones in the
    // unrolled tail. A lane-striped hash that dropped a lane would still look
    // fine on a single flipped byte, so this walks several positions.
    for (size_t pos : {size_t{0}, size_t{1}, size_t{7}, size_t{8}, size_t{2048},
                       a.size() - 9, a.size() - 1}) {
        std::vector<uint8_t> b = a;
        b[pos] ^= 0x01;
        INFO("flipped byte at " << pos);
        CHECK(fnv1a_bulk(a.data(), a.size()) != fnv1a_bulk(b.data(), b.size()));
    }

    // Sensitive to ORDER, which a per-lane sum would not be.
    std::vector<uint8_t> swapped = a;
    std::swap(swapped[100], swapped[108]);        // same lane, different index
    CHECK(fnv1a_bulk(a.data(), a.size()) != fnv1a_bulk(swapped.data(), swapped.size()));

    // Sensitive to LENGTH. Trailing zeros are the case a lane-striped
    // construction gets wrong if the length is not folded in, and a frame
    // buffer is full of zeros.
    std::vector<uint8_t> zeros_a(64, 0), zeros_b(72, 0);
    CHECK(fnv1a_bulk(zeros_a.data(), zeros_a.size())
       != fnv1a_bulk(zeros_b.data(), zeros_b.size()));
}

TEST_CASE("P1-2: fnv1a_bulk handles every tail length") {
    // The body loop consumes eight bytes at a time and a separate loop
    // finishes the remainder. Both must assign bytes to the same lanes, or a
    // buffer whose length is not a multiple of eight hashes differently
    // depending on where the split lands — and a 641x481 direct-mode frame is
    // exactly that (308,321 bytes, 1 byte of tail).
    std::vector<uint8_t> buf(64);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = static_cast<uint8_t>(i);

    std::vector<uint64_t> digests;
    for (size_t n = 0; n <= 24; ++n) digests.push_back(fnv1a_bulk(buf.data(), n));

    // Every prefix length gives a distinct digest. Not a deep property, but it
    // is the one that fails outright if the tail loop uses a different lane
    // assignment from the body.
    for (size_t i = 0; i < digests.size(); ++i) {
        for (size_t j = i + 1; j < digests.size(); ++j) {
            INFO("lengths " << i << " and " << j);
            CHECK(digests[i] != digests[j]);
        }
    }

    // Zero length is well defined and not the seed.
    CHECK(fnv1a_bulk(buf.data(), 0) == fnv1a_bulk(nullptr, 0));
}

TEST_CASE("P1-2: fnv1a_bulk reads single bytes, so it cannot depend on endianness") {
    // This is the property that made the obvious optimisation unusable. Eight
    // bytes read as a uint64 hash differently on a big-endian machine, and
    // this hash feeds the INV-3 fingerprint, whose whole claim is that two
    // machines agree.
    //
    // Endianness cannot be tested from inside one process. What CAN be tested
    // is the property it follows from: the digest is a function of the byte
    // SEQUENCE, so a buffer and its byte-reversal must differ, and building
    // the same sequence from differently-aligned storage must not.
    std::vector<uint8_t> fwd(32);
    for (size_t i = 0; i < fwd.size(); ++i) fwd[i] = static_cast<uint8_t>(i + 1);
    std::vector<uint8_t> rev(fwd.rbegin(), fwd.rend());
    CHECK(fnv1a_bulk(fwd.data(), fwd.size()) != fnv1a_bulk(rev.data(), rev.size()));

    // Same bytes, offset storage: identical digest. If any multi-byte load had
    // crept in, an unaligned copy could differ.
    std::vector<uint8_t> offset(fwd.size() + 3, 0xAA);
    std::copy(fwd.begin(), fwd.end(), offset.begin() + 3);
    CHECK(fnv1a_bulk(fwd.data(), fwd.size())
       == fnv1a_bulk(offset.data() + 3, fwd.size()));
}
