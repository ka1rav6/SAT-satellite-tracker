// core/result.hpp — the error type used everywhere loading or validation can
// fail.
//
// Why not exceptions?
//   * Scenario validation (design §7.5) must report an error that cites the
//     offending file, line and specification row. That is a *value*, and
//     carrying values is what a Result does well.
//   * Model loading (design §11.3, INV-7) must degrade to a classical fallback,
//     not unwind. `if (!r) { warn(); use_classical(); }` is the shape we want.
//   * The frame loop is allocation-free (INV-4) and must not need a landing
//     pad. Nothing in the hot path returns a Result; everything that can fail
//     does so at load time.
//
// Why not std::expected? It is C++23 and the project is pinned to C++20
// (AGENTS.md §1). This is the ~80-line subset we actually use, and it can be
// swapped for std::expected in one file when the standard moves.

#pragma once

#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace sat {

// ---------------------------------------------------------------------------
// Error — a human-readable message, already formatted for the user.
//
// A plain string rather than an enum + code, because every failure in this
// program is reported to a person reading a terminal, and the useful content
// ("gimbal.max_pan_dps = 14.0 is outside the permitted range [5.0, 10.0]
// (specification row 13)") does not fit in an enum.
// ---------------------------------------------------------------------------
struct Error {
    std::string message;
};

/// Tag type so `return Err("...")` works for any Result<T> without naming T.
struct ErrTag {
    std::string message;
};

/// Build an error. Deliberately short: it appears at every validation site.
[[nodiscard]] inline ErrTag Err(std::string msg) { return ErrTag{std::move(msg)}; }

// ---------------------------------------------------------------------------
// Result<T> — either a T or an Error.
//
// Marked [[nodiscard]] so a forgotten check is a compiler warning. That matters
// most in scenario loading, where ignoring a failure would run the simulation
// on a half-parsed config.
// ---------------------------------------------------------------------------
template <typename T>
class [[nodiscard]] Result {
public:
    // Implicit conversions on purpose: `return Ok(x);` and `return Err("...");`
    // should both just work at a call site that returns Result<T>.
    Result(T value) : store_(std::move(value)) {}            // NOLINT(google-explicit-constructor)
    Result(ErrTag e) : store_(Error{std::move(e.message)}) {} // NOLINT(google-explicit-constructor)
    Result(Error e) : store_(std::move(e)) {}                 // NOLINT(google-explicit-constructor)

    [[nodiscard]] bool has_value() const noexcept { return store_.index() == 0; }
    explicit operator bool() const noexcept { return has_value(); }

    // Accessors. No runtime check in release: calling these on the wrong
    // alternative is a programming error, and std::get already traps in debug.
    [[nodiscard]] T&       value()       & { return std::get<0>(store_); }
    [[nodiscard]] const T& value() const & { return std::get<0>(store_); }
    [[nodiscard]] T&&      value()      && { return std::get<0>(std::move(store_)); }

    [[nodiscard]] T&       operator*()       & { return value(); }
    [[nodiscard]] const T& operator*() const & { return value(); }
    [[nodiscard]] T*       operator->()        { return &value(); }
    [[nodiscard]] const T* operator->() const  { return &value(); }

    /// The message, or an empty string if this holds a value.
    [[nodiscard]] const std::string& error() const {
        static const std::string kNone{};
        return has_value() ? kNone : std::get<1>(store_).message;
    }

    /// The value if present, otherwise the supplied default. Useful for
    /// optional settings where a failure is not fatal.
    [[nodiscard]] T value_or(T fallback) const& {
        return has_value() ? value() : std::move(fallback);
    }

private:
    std::variant<T, Error> store_;
};

// ---------------------------------------------------------------------------
// Result<void> — for operations that either succeed or explain why not.
// ---------------------------------------------------------------------------
template <>
class [[nodiscard]] Result<void> {
public:
    Result() = default;
    Result(ErrTag e) : err_(Error{std::move(e.message)}), ok_(false) {}  // NOLINT
    Result(Error e) : err_(std::move(e)), ok_(false) {}                  // NOLINT

    [[nodiscard]] bool has_value() const noexcept { return ok_; }
    explicit operator bool() const noexcept { return ok_; }
    [[nodiscard]] const std::string& error() const noexcept { return err_.message; }

private:
    Error err_{};
    bool  ok_ = true;
};

using Status = Result<void>;

/// Build a success. `Ok()` with no argument yields a successful Status.
template <typename T>
[[nodiscard]] Result<std::decay_t<T>> Ok(T&& value) {
    return Result<std::decay_t<T>>(std::forward<T>(value));
}
[[nodiscard]] inline Status Ok() { return Status{}; }

}  // namespace sat
