// core/arena.hpp — bump allocator.
//
// INV-4 (design §2): "After scenario load, no allocation occurs during frame
// processing." Perception still needs scratch space -- two summed-area tables,
// a filtered image, a top-hat image, a run list -- and that space is different
// sizes for different sensor resolutions, so it cannot all be a fixed array.
//
// An arena resolves that. Memory is reserved ONCE at startup (design §6.1 step
// A4). Inside a frame, "allocating" is a pointer bump; "freeing" is resetting
// the pointer to zero at the end of the frame. No malloc, no free, no
// fragmentation, no destructor calls, and allocation is about two instructions.
//
// The arena hands out raw, uninitialised, POD storage. It deliberately does NOT
// run constructors or destructors: everything it is used for here is a span of
// arithmetic types. If you need an object with a destructor inside a frame,
// something has gone wrong with the design, not with this class.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// Arena — a fixed block of bytes plus a bump pointer.
// ---------------------------------------------------------------------------
class Arena {
public:
    Arena() = default;

    /// Reserve `bytes` of storage. This is the allocation INV-4 permits, and it
    /// must happen before the loop starts. Calling reserve() again releases the
    /// old block, so it is a load-time operation only.
    explicit Arena(size_t bytes) { reserve(bytes); }

    void reserve(size_t bytes) {
        // Over-align to 64 bytes so every sub-allocation can be cache-line and
        // AVX2 aligned without the caller thinking about it.
        storage_ = std::make_unique<std::byte[]>(bytes + kAlign);
        base_    = align_up(storage_.get(), kAlign);
        // The extra kAlign bytes we asked for cover whatever alignment moved us.
        capacity_ = bytes;
        offset_   = 0;
        high_water_ = 0;
    }

    /// Allocate `count` objects of type T. Returns a span so the caller carries
    /// the length with the pointer -- most out-of-bounds bugs in image kernels
    /// are a pointer that lost its length.
    ///
    /// On exhaustion this returns an EMPTY span rather than throwing. Callers in
    /// the frame path check `!s.empty()`; the sizing is validated once at
    /// startup by a dry run, so an empty span in production means the arena was
    /// mis-sized, which the metrics layer reports rather than crashing a demo.
    template <typename T>
    [[nodiscard]] std::span<T> alloc(size_t count) noexcept {
        static_assert(std::is_trivially_default_constructible_v<T>,
                      "Arena hands out raw storage; T must not need a constructor");
        static_assert(std::is_trivially_destructible_v<T>,
                      "Arena never runs destructors; T must not need one");
        if (count == 0) return {};

        const size_t align = alignof(T) < kMinAlign ? kMinAlign : alignof(T);
        size_t       cur   = reinterpret_cast<size_t>(base_) + offset_;
        const size_t pad   = (align - (cur % align)) % align;
        const size_t need  = pad + count * sizeof(T);

        if (offset_ + need > capacity_) {
            ++exhaustions_;
            return {};
        }
        std::byte* p = base_ + offset_ + pad;
        offset_ += need;
        if (offset_ > high_water_) high_water_ = offset_;
        return std::span<T>(reinterpret_cast<T*>(p), count);
    }

    /// Allocate and zero. Slightly slower; use it where the kernel assumes a
    /// clean buffer (summed-area table borders, accumulator arrays).
    template <typename T>
    [[nodiscard]] std::span<T> alloc_zeroed(size_t count) noexcept {
        auto s = alloc<T>(count);
        if (!s.empty()) std::memset(s.data(), 0, s.size_bytes());
        return s;
    }

    /// Release everything. O(1): it is one store. Called once per frame.
    void reset() noexcept { offset_ = 0; }

    // --- introspection, used by the metrics report and the sizing tests -----
    [[nodiscard]] size_t capacity()   const noexcept { return capacity_; }
    [[nodiscard]] size_t used()       const noexcept { return offset_; }
    [[nodiscard]] size_t high_water() const noexcept { return high_water_; }
    [[nodiscard]] uint64_t exhaustions() const noexcept { return exhaustions_; }

    /// Current bump position, for ScopedArena.
    [[nodiscard]] size_t mark() const noexcept { return offset_; }
    void release_to(size_t m) noexcept { offset_ = m; }

private:
    static constexpr size_t kAlign    = 64;   // cache line / AVX-512 friendly
    static constexpr size_t kMinAlign = 32;   // AVX2 friendly for every sub-alloc

    static std::byte* align_up(std::byte* p, size_t a) noexcept {
        const size_t v = reinterpret_cast<size_t>(p);
        return reinterpret_cast<std::byte*>((v + a - 1) & ~(a - 1));
    }

    std::unique_ptr<std::byte[]> storage_;
    std::byte* base_       = nullptr;
    size_t     capacity_   = 0;
    size_t     offset_     = 0;
    size_t     high_water_ = 0;
    uint64_t   exhaustions_ = 0;
};

// ---------------------------------------------------------------------------
// ScopedArena — RAII save/restore of the bump pointer.
//
// Lets a nested stage carve temporary space out of the frame arena and give it
// back at the end of its own scope, without waiting for the frame reset. Useful
// where one stage needs a large scratch buffer that a later stage would
// otherwise be unable to fit.
// ---------------------------------------------------------------------------
class ScopedArena {
public:
    explicit ScopedArena(Arena& a) noexcept : arena_(a), mark_(a.mark()) {}
    ~ScopedArena() { arena_.release_to(mark_); }

    ScopedArena(const ScopedArena&)            = delete;
    ScopedArena& operator=(const ScopedArena&) = delete;

    [[nodiscard]] Arena& get() noexcept { return arena_; }
    operator Arena&() noexcept { return arena_; }   // NOLINT(google-explicit-constructor)

private:
    Arena& arena_;
    size_t mark_;
};

// ---------------------------------------------------------------------------
// ArenaSet — the four arenas design §6.1 step A4 specifies.
//
// Sizes are defaults; the scenario loader may enlarge them for an unusually
// large sensor. The persistent arena is never reset during a run; the frame
// arena is reset at the top of every frame.
// ---------------------------------------------------------------------------
struct ArenaSet {
    Arena persistent;   ///< world, masks, bias tables -- lives for the whole run
    Arena frame;        ///< per-frame scratch; reset() every frame
    Arena video;        ///< decoded-frame ring backing store (video modes only)
    Arena report;       ///< metric accumulation and report generation

    static constexpr size_t kDefaultPersistentBytes = 24u << 20;   // 24 MB
    static constexpr size_t kDefaultFrameBytes      =  8u << 20;   //  8 MB
    static constexpr size_t kDefaultVideoBytes      = 48u << 20;   // 48 MB
    static constexpr size_t kDefaultReportBytes     =  8u << 20;   //  8 MB

    void reserve_defaults() {
        persistent.reserve(kDefaultPersistentBytes);
        frame.reserve(kDefaultFrameBytes);
        video.reserve(kDefaultVideoBytes);
        report.reserve(kDefaultReportBytes);
    }
};

// ---------------------------------------------------------------------------
// Allocation trap (INV-4, CP 14.3).
//
// In debug builds the program installs a global operator new that asserts when
// called while a frame is in flight. This flag is what it checks. It lives here
// rather than in the trap's own translation unit so that any code can set it
// without pulling in the trap.
//
// Not atomic on purpose: the simulation is single-threaded by design decision
// 11, and making it atomic would cost a fence on a path we measure in
// microseconds.
// ---------------------------------------------------------------------------
extern bool g_in_frame;

/// RAII guard that marks the frame-processing window for the allocation trap.
class FrameScope {
public:
    FrameScope()  noexcept { g_in_frame = true;  }
    ~FrameScope() noexcept { g_in_frame = false; }
    FrameScope(const FrameScope&)            = delete;
    FrameScope& operator=(const FrameScope&) = delete;
};

}  // namespace sat
