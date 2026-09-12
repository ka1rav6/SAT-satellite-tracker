// core/ring.hpp — a fixed-capacity circular buffer.
//
// Two very different consumers, which is why the interface looks the way it
// does:
//
//   1. The gimbal transport delay line (design §10.3). The controller's command
//      does not reach the motor for `latency_s`; the plant pushes each command
//      and reads the one from `d` steps ago. That is `at_back(d)`.
//
//   2. Rolling metric windows -- the last N innovations for the NIS check, the
//      last N frame times for a percentile estimate.
//
// Capacity is a template parameter and must be a power of two, so the modulo is
// a mask. There is no allocation and no growth: a ring that is full overwrites
// its oldest element, which is exactly what both consumers want.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace sat {

template <typename T, size_t N>
class Ring {
    static_assert(N > 0 && (N & (N - 1)) == 0, "Ring capacity must be a power of two");

public:
    Ring() = default;

    /// Fill the whole ring with one value. The delay line needs this: before the
    /// first command arrives, "the command from 3 steps ago" must be a defined
    /// value (zero rate), not garbage.
    void fill(const T& v) {
        data_.fill(v);
        head_ = 0;
        size_ = N;
    }

    void clear() noexcept { head_ = 0; size_ = 0; }

    /// Append, overwriting the oldest element once full.
    void push(const T& v) noexcept {
        data_[head_] = v;
        head_ = (head_ + 1) & kMask;
        if (size_ < N) ++size_;
    }

    /// The element `d` pushes ago. at_back(0) is the most recent push,
    /// at_back(1) the one before it, and so on.
    ///
    /// Requesting further back than the ring holds clamps to the oldest
    /// element rather than reading uninitialised memory. For the delay line
    /// that is the right behaviour at startup: before enough history exists,
    /// the motor sees the oldest command available.
    [[nodiscard]] const T& at_back(size_t d) const noexcept {
        if (size_ == 0) return data_[0];
        if (d >= size_) d = size_ - 1;
        return data_[(head_ + N - 1 - d) & kMask];
    }

    [[nodiscard]] const T& newest() const noexcept { return at_back(0); }
    [[nodiscard]] const T& oldest() const noexcept { return at_back(size_ ? size_ - 1 : 0); }

    [[nodiscard]] size_t size()     const noexcept { return size_; }
    [[nodiscard]] bool   empty()    const noexcept { return size_ == 0; }
    [[nodiscard]] bool   full()     const noexcept { return size_ == N; }
    [[nodiscard]] static constexpr size_t capacity() noexcept { return N; }

    /// Visit every stored element, oldest first. A callback rather than
    /// iterators because the storage is not contiguous in logical order and a
    /// wrapping iterator would be more code than it saves.
    template <typename F>
    void for_each(F&& f) const {
        for (size_t i = 0; i < size_; ++i) f(at_back(size_ - 1 - i));
    }

private:
    static constexpr size_t kMask = N - 1;

    std::array<T, N> data_{};
    size_t           head_ = 0;   ///< index of the next slot to write
    size_t           size_ = 0;   ///< how many slots hold real data
};

}  // namespace sat
