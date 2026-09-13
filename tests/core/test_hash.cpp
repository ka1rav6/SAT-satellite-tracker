// tests/core/test_hash.cpp — CP 0.4 / CP 2.5 groundwork.
//
// The reproducibility fingerprint is only useful if it is (a) stable for
// identical input and (b) sensitive to the things that matter. Both directions
// are tested here.

#include <doctest/doctest.h>

#include "core/hash.hpp"

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
