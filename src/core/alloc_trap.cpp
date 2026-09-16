// core/alloc_trap.cpp — INV-4's enforcement (CP 14.3).
//
// ---------------------------------------------------------------------------
// WHY A GLOBAL operator new, AND WHY ONLY IN DEBUG
// ---------------------------------------------------------------------------
// INV-4 says the steady state allocates nothing. Every stage has been written
// to honour that — the frame arena, the fixed-capacity rings, Eigen's
// fixed-size matrices, the reserved vectors — and every one of those is a
// PROMISE. A promise about allocation is the kind that decays silently: a
// std::function here, a std::string concatenation in a log line there, a
// vector that outgrows its reserve on the one frame that had 30 candidates.
// Nothing goes wrong immediately. The frame budget just drifts, and the p99
// grows a tail nobody can attribute.
//
// The only way to hold the line is to make the compiler and the runtime check
// it. This replaces the global operator new with one that aborts if it is
// called while a frame is in flight, which turns "we believe this allocates
// nothing" into a test that fails loudly the first time it stops being true.
//
// DEBUG ONLY, and that is not a compromise. The trap costs a branch per
// allocation, which is irrelevant, but replacing operator new globally in a
// shipping binary is a liability — any third-party code that allocates during
// a frame for a legitimate reason (OpenCV's decoder, ONNX Runtime's arena)
// would abort the program rather than being slightly slower than we would
// like. The Release build is the one that runs; the Debug build is the one
// that checks. `just test-debug` runs the whole suite under it, which is why
// the trap is worth having at all.
//
// ---------------------------------------------------------------------------
// WHAT COUNTS AS "IN A FRAME"
// ---------------------------------------------------------------------------
// core/arena.hpp's FrameScope marks the window, and the window is deliberately
// narrow: it covers the per-frame processing and NOT the setup that precedes a
// run. Building the world, sizing the arena, reserving the metric series and
// opening the logs all allocate, and all of them should — INV-4 is about the
// STEADY state, not about a program that never calls malloc.

#include "core/arena.hpp"

#include <cstdio>
#include <cstdlib>
#include <new>

#if defined(SAT_ALLOC_TRAP)

namespace {

// Set once a violation has been reported. Without it the abort path itself can
// allocate — fprintf may, on some libcs — and the program recurses until the
// stack runs out, which replaces a clear message with a mystery crash.
bool g_reporting = false;

void trip(std::size_t n) noexcept {
    if (sat::g_in_frame && !g_reporting) {
        g_reporting = true;
        std::fprintf(stderr,
                     "\nINV-4 VIOLATION: %zu bytes allocated during frame "
                     "processing.\n"
                     "The steady state must not allocate. Run this under a "
                     "debugger and break on\n"
                     "operator new to find it; the usual causes are a vector "
                     "outgrowing its reserve,\n"
                     "a std::string built in a log line, or a std::function "
                     "capturing by value.\n", n);
        std::abort();
    }
}

}  // namespace

void* operator new(std::size_t n) {
    trip(n);
    if (n == 0) n = 1;
    void* p = std::malloc(n);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new[](std::size_t n) { return ::operator new(n); }

void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    trip(n);
    if (n == 0) n = 1;
    return std::malloc(n);
}
void* operator new[](std::size_t n, const std::nothrow_t& t) noexcept {
    return ::operator new(n, t);
}

// The deletes are NOT trapped. Freeing during a frame is fine — it does not
// grow the heap and it does not take the allocator's slow path — and trapping
// it would fire on every temporary that happened to be destroyed inside the
// window, including ones allocated long before it.
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }

#endif  // SAT_ALLOC_TRAP
