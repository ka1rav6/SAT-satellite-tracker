// engine/decode_thread.hpp — CP 8.1.
//
// "Decode thread + ring buffer, greyscale at decode. Accept when: the main loop
//  never waits; a 2000x2000 clip decodes at >60 fps."
//
// ---------------------------------------------------------------------------
// THE ONLY EXTRA THREAD IN THE PROGRAM, AND WHY IT DOES NOT BREAK INV-3
// ---------------------------------------------------------------------------
// Design §8.3 requirement 1: "Decode on a separate thread, filling a ring
// buffer. A 2000x2000 H.264 frame costs 5-10 ms; inline decode would consume a
// third of the budget. This is the ONLY extra thread and it is safe because
// decode produces identical frames in identical order regardless of timing."
//
// That last clause is the whole argument and it is worth being precise about.
// INV-3 requires bit-exact reproducibility. A thread normally destroys that,
// because the interleaving varies run to run. Here it does not, because the
// only thing crossing the boundary is a SEQUENCE of decoded frames, and that
// sequence is a pure function of the file:
//
//   * the decoder is single-threaded internally (see open()), so its own
//     output does not depend on scheduling;
//   * frames are pushed and popped in order, never dropped, never reordered —
//     a full ring BLOCKS the decoder rather than discarding, which is the
//     opposite of what a real-time video pipeline would do and exactly right
//     here, because a dropped frame would change the result;
//   * the consumer's timing affects only HOW LONG it waits, never WHAT it
//     receives.
//
// If any of those three stopped being true, INV-3 would be gone and the
// reproducibility gate would catch it — that gate runs on video scenarios too.
//
// ---------------------------------------------------------------------------
// GREYSCALE AT DECODE
// ---------------------------------------------------------------------------
// Requirement 2. Converting per frame in perception would put a 300 KB colour
// conversion inside the 0.85 ms budget, and would also mean every later stage
// had to know whether the source was colour. The ring carries 8-bit grey and
// nothing downstream can tell where it came from.

#pragma once

#include "core/result.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// DecodedFrame — one slot of the ring.
// ---------------------------------------------------------------------------
struct DecodedFrame {
    std::vector<uint8_t> grey;      ///< width*height, 8-bit
    int     width  = 0;
    int     height = 0;
    int64_t index  = 0;             ///< sequential, counting frames we KEPT

    /// Container timestamp in seconds (design §8.3 requirement 9). Not
    /// index/fps: a variable-frame-rate file would put every metric on a wrong
    /// time axis, and CP 8.8 ships a VFR clip specifically to catch that.
    double  timestamp_s = 0.0;

    /// True when the decoder skipped one or more corrupt frames immediately
    /// before this one. Requirement 8 says skip, log and continue; the count
    /// is carried so the run can REPORT how much of the file it could not read
    /// rather than silently shortening.
    int     skipped_before = 0;
};

// ---------------------------------------------------------------------------
// DecodeThread — a bounded, blocking, single-producer single-consumer queue
// fed by a decoder running on its own thread.
// ---------------------------------------------------------------------------
class DecodeThread {
public:
    // Both defined out of line, in decode_thread.cpp. Impl is incomplete here
    // (it holds a cv::VideoCapture, which must not leak OpenCV into every
    // translation unit that mentions a video source), so the compiler cannot
    // generate a destructor for unique_ptr<Impl> in this header — and the
    // CONSTRUCTOR needs one too, for its exception-cleanup path.
    DecodeThread();
    ~DecodeThread();
    DecodeThread(const DecodeThread&) = delete;
    DecodeThread& operator=(const DecodeThread&) = delete;

    /// Open the file and start decoding. `capacity` is the ring depth; 8 is
    /// §8.3's figure, which at 30 fps is a quarter-second of slack — enough to
    /// absorb a slow frame without letting the decoder run so far ahead that a
    /// seek or an error is discovered long after the fact.
    [[nodiscard]] Status open(const std::filesystem::path& path, int capacity = 8);

    /// Pop the next frame. Blocks until one is available or the file ends.
    /// Returns false at end of stream — which is a CLEAN termination
    /// (requirement 7), not an error.
    [[nodiscard]] bool pop(DecodedFrame& out);

    /// Stop decoding and join. Safe to call twice; the destructor calls it.
    void close();

    // --- what the container said, probed once at open ----------------------
    [[nodiscard]] int    width()  const noexcept { return width_; }
    [[nodiscard]] int    height() const noexcept { return height_; }
    [[nodiscard]] double fps()    const noexcept { return fps_; }
    [[nodiscard]] const std::string& backend() const noexcept { return backend_; }

    /// Frames the decoder could not read and skipped. Reported at shutdown.
    [[nodiscard]] int64_t skipped() const noexcept { return skipped_.load(); }
    /// Frames delivered.
    [[nodiscard]] int64_t decoded() const noexcept { return decoded_.load(); }

    /// How often the CONSUMER had to wait for the decoder. CP 8.1's criterion
    /// is "the main loop never waits", and the honest way to check that is to
    /// count the times it did rather than to assert it in a comment.
    [[nodiscard]] int64_t consumer_waits() const noexcept { return waits_.load(); }

private:
    void run();                       ///< the decode loop, on its own thread

    struct Impl;                      ///< holds cv::VideoCapture, out of this header
    std::unique_ptr<Impl> impl_;

    std::vector<DecodedFrame> ring_;
    size_t head_ = 0, tail_ = 0, size_ = 0;

    std::mutex              mu_;
    std::condition_variable not_empty_, not_full_;
    std::thread             thread_;
    std::atomic<bool>       stop_{false};
    bool                    eof_ = false;

    int    width_  = 0, height_ = 0;
    double fps_    = 0.0;
    std::string backend_;
    std::atomic<int64_t> skipped_{0}, decoded_{0}, waits_{0};
};

}  // namespace sat
