// core/triple_buffer.hpp — lock-free hand-off from the simulation thread to the
// display thread.
//
// Design §6.2 step B30/B31 and CP 2.3: the simulation publishes a snapshot every
// frame and the GUI reads the newest one, *and slowing the display down must not
// slow the simulation down*. That last requirement rules out a mutex (the
// producer would block behind a slow consumer) and a queue (an unbounded backlog
// if the consumer is slower, which it will be -- 30..500 Hz sim versus a 60 Hz
// monitor).
//
// A triple buffer is the standard answer:
//
//      write  -- the producer is filling this one
//      ready  -- the newest fully-written one, waiting to be picked up
//      read   -- the consumer is looking at this one
//
// Publishing swaps `write` with `ready`. Acquiring swaps `read` with `ready`.
// Both are a single atomic exchange, so neither side ever waits, and the
// consumer always gets the newest complete snapshot -- dropping intermediate
// frames, which for a display is exactly right.
//
// The buffers are constructed once and never reallocated, so a snapshot type
// that owns storage (e.g. a downsampled preview image) must size that storage
// at construction. That keeps INV-4 intact: publishing allocates nothing.

#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <utility>

namespace sat {

template <typename T>
class TripleBuffer {
public:
    TripleBuffer() = default;

    /// Construct all three buffers from a prototype. Use this when T needs
    /// pre-sized storage: pass a T that has already reserved what it needs.
    explicit TripleBuffer(const T& prototype) : slots_{prototype, prototype, prototype} {}

    /// Re-initialise all three slots from a prototype and rewind the indices.
    ///
    /// A separate method rather than assignment, because the atomics make this
    /// type non-copyable and non-movable — which is correct (an atomic's value
    /// is meaningless to copy) but means a triple buffer cannot be replaced
    /// wholesale once constructed.
    ///
    /// MUST NOT be called while a consumer is reading. It is a setup operation:
    /// once per run at build time, or between sweep runs inside one process.
    void reset(const T& prototype) {
        for (auto& s : slots_) s = prototype;
        write_idx_ = 0;
        read_idx_  = 2;
        ready_.store(1, std::memory_order_relaxed);
        published_.store(0, std::memory_order_relaxed);
        consumed_ = 0;
    }

    /// The buffer the producer may write into. Valid until publish().
    [[nodiscard]] T& write_slot() noexcept { return slots_[write_idx_]; }

    /// Make the current write buffer visible to the consumer and take ownership
    /// of whatever was previously ready.
    ///
    /// memory_order_acq_rel: everything written to the slot before this call
    /// must be visible to a consumer that subsequently acquires it (release),
    /// and we must see the consumer's prior stores to the slot we take back
    /// (acquire).
    void publish() noexcept {
        const uint32_t prev = ready_.exchange(write_idx_ | kFreshBit, std::memory_order_acq_rel);
        write_idx_ = prev & kIndexMask;
        published_.fetch_add(1, std::memory_order_relaxed);
    }

    /// Take the newest published snapshot if there is one newer than what the
    /// consumer already holds. Returns false if nothing new has been published,
    /// in which case the consumer should keep drawing what it has.
    [[nodiscard]] bool acquire() noexcept {
        if ((ready_.load(std::memory_order_acquire) & kFreshBit) == 0) return false;
        const uint32_t prev = ready_.exchange(read_idx_, std::memory_order_acq_rel);
        read_idx_ = prev & kIndexMask;
        ++consumed_;
        return true;
    }

    /// The buffer the consumer may read. Valid until the next acquire().
    [[nodiscard]] const T& read_slot() const noexcept { return slots_[read_idx_]; }

    /// How many snapshots the producer published and the consumer actually saw.
    /// The difference is the drop count, which the GUI reports so a viewer can
    /// see that the simulation is outrunning the display rather than stalling.
    [[nodiscard]] uint64_t published_count() const noexcept {
        return published_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t consumed_count() const noexcept { return consumed_; }

private:
    static constexpr uint32_t kIndexMask = 0x3u;
    static constexpr uint32_t kFreshBit  = 0x4u;   ///< set when `ready` holds unseen data

    std::array<T, 3> slots_{};

    // Indices 0, 1, 2 across the three roles; they permute but never collide.
    uint32_t              write_idx_ = 0;   ///< producer thread only
    uint32_t              read_idx_  = 2;   ///< consumer thread only
    std::atomic<uint32_t> ready_{1};        ///< shared: index | fresh bit

    std::atomic<uint64_t> published_{0};
    uint64_t              consumed_ = 0;
};

}  // namespace sat
