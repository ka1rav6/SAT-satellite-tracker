// engine/decode_thread.cpp

#include "engine/decode_thread.hpp"

#if SAT_HAVE_OPENCV
#  include <opencv2/core.hpp>
#  include <opencv2/imgproc.hpp>
#  include <opencv2/videoio.hpp>
#endif

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

namespace sat {

#if SAT_HAVE_OPENCV

struct DecodeThread::Impl {
    cv::VideoCapture cap;
    cv::Mat          bgr;
    cv::Mat          grey;
};

DecodeThread::DecodeThread() = default;
DecodeThread::~DecodeThread() { close(); }

int DecodeThread::default_threads() noexcept {
    const unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) return 1;                      // unknowable: be conservative
    return static_cast<int>(hw < 4u ? hw : 4u);
}

Status DecodeThread::open(const std::filesystem::path& path, int capacity, int threads) {
    close();
    impl_ = std::make_unique<Impl>();

    // ------------------------------------------------------------------
    // DECODER WORKER THREADS — P0-3.
    //
    // This used to force 1, both ways, permanently. See the long note in the
    // header for why one of the two stated reasons was real (a deadlock on a
    // malformed clip, now handled by pop()'s watchdog) and the other was not
    // (threaded H.264 decode is bit-exact for conforming streams, measured on
    // every committed clip).
    //
    // The environment variable is set before opening because OpenCV reads it
    // when it constructs the backend; CAP_PROP_N_THREADS is set after, because
    // some backends only accept it on an open capture. Both, because neither
    // is reliable alone. 70 is CAP_PROP_N_THREADS numerically — OpenCV 4.6
    // does not define the enum, and hard-coding the number with this comment
    // beats failing to compile on the version CI installs.
    // ------------------------------------------------------------------
    threads_ = (threads > 0) ? threads : default_threads();
    const std::string nthreads = std::to_string(threads_);
#if defined(_WIN32)
    _putenv_s("OPENCV_FFMPEG_THREADS", nthreads.c_str());
#else
    setenv("OPENCV_FFMPEG_THREADS", nthreads.c_str(), 1);
#endif
    if (!impl_->cap.open(path.string(), cv::CAP_ANY)) {
        impl_.reset();
        return Err("cannot open video '" + path.string() +
                   "' (unsupported codec, or the file is not a video)");
    }
    impl_->cap.set(70 /* CAP_PROP_N_THREADS */, threads_);

    width_   = static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_WIDTH));
    height_  = static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    fps_     = impl_->cap.get(cv::CAP_PROP_FPS);
    backend_ = impl_->cap.getBackendName();

    if (width_ <= 0 || height_ <= 0) {
        impl_.reset();
        return Err("video '" + path.string() + "' reports a " +
                   std::to_string(width_) + "x" + std::to_string(height_) +
                   " frame size, which cannot be decoded");
    }

    ring_.assign(static_cast<size_t>(capacity > 0 ? capacity : 8), DecodedFrame{});
    head_ = tail_ = size_ = 0;
    eof_ = false;
    stop_.store(false);
    stalled_.store(false);
    skipped_.store(0);
    decoded_.store(0);
    waits_.store(0);

    thread_ = std::thread([this] { run(); });
    return Ok();
}

void DecodeThread::run() {
    int64_t index = 0;
    int     skipped_run = 0;   ///< confirmed gap immediately before this frame
    int     probe_run   = 0;   ///< failed reads not yet known to be a gap

    for (;;) {
        if (stop_.load()) break;

        bool got = false;
        try {
            got = impl_->cap.read(impl_->bgr) && !impl_->bgr.empty();
        } catch (const cv::Exception&) {
            // §8.3 requirement 8: "Never crash on a corrupt frame — skip, log,
            // continue." OpenCV can throw from inside read() on a malformed
            // packet, and an exception escaping this thread would call
            // std::terminate and take the whole process with it.
            got = false;
        }

        if (!got) {
            // Ambiguous: end of file, or one unreadable frame. Distinguishing
            // them reliably is not possible through this API, so the rule is to
            // retry a bounded number of times and then treat it as EOF. A
            // truncated file (CP 8.8 ships two) hits this immediately and
            // terminates cleanly; a single corrupt frame mid-file is skipped
            // and decoding continues.
            //
            // The retries are NOT counted as skipped frames yet. They are
            // counted only once another frame actually decodes, which is what
            // makes them a mid-file gap rather than the end of the stream.
            //
            // The first version counted immediately, and every clip in the
            // suite — including the clean ones — reported exactly 8 skipped
            // frames: the EOF probe itself. A diagnostic that fires identically
            // on healthy and damaged input measures nothing, and this one goes
            // into run.json where it would have been read as data loss.
            if (++probe_run <= 8) continue;
            break;
        }
        if (probe_run > 0) {
            // A real gap: frames we could not read, followed by one we could.
            skipped_.fetch_add(probe_run);
            skipped_run = probe_run;
            probe_run   = 0;
        }

        // --- greyscale at decode (requirement 2) --------------------------
        if (impl_->bgr.channels() == 1) {
            impl_->grey = impl_->bgr;
        } else {
            cv::cvtColor(impl_->bgr, impl_->grey, cv::COLOR_BGR2GRAY);
        }

        // --- container timestamp (requirement 9) --------------------------
        // CAP_PROP_POS_MSEC is the container's own clock. It is read AFTER the
        // frame, so it is the time of the frame just decoded. A container that
        // does not provide it returns 0 forever, in which case fall back to the
        // counted index — stated here rather than silently, because a VFR file
        // whose timestamps are missing really does put metrics on a nominal
        // time axis and the run.json should not imply otherwise.
        double ts = impl_->cap.get(cv::CAP_PROP_POS_MSEC) * 1e-3;
        if (!(ts > 0.0) && fps_ > 0.0) ts = static_cast<double>(index) / fps_;

        std::unique_lock<std::mutex> lk(mu_);
        // A FULL RING BLOCKS THE DECODER. It does not drop. Dropping would make
        // the output depend on timing and INV-3 would be gone; see the header.
        not_full_.wait(lk, [this] { return size_ < ring_.size() || stop_.load(); });
        if (stop_.load()) break;

        DecodedFrame& slot = ring_[head_];
        const size_t n = static_cast<size_t>(impl_->grey.total());
        slot.grey.resize(n);
        // Row by row: cv::Mat may be padded, and a flat copy of a padded Mat
        // would shear the image by the stride difference.
        for (int y = 0; y < impl_->grey.rows; ++y) {
            const uint8_t* src = impl_->grey.ptr<uint8_t>(y);
            std::copy(src, src + impl_->grey.cols,
                      slot.grey.begin() + static_cast<ptrdiff_t>(y) * impl_->grey.cols);
        }
        slot.width  = impl_->grey.cols;
        slot.height = impl_->grey.rows;
        slot.index  = index++;
        slot.timestamp_s    = ts;
        slot.skipped_before = skipped_run;
        skipped_run = 0;

        head_ = (head_ + 1) % ring_.size();
        ++size_;
        decoded_.fetch_add(1);
        lk.unlock();
        not_empty_.notify_one();
    }

    {
        std::lock_guard<std::mutex> lk(mu_);
        eof_ = true;
    }
    not_empty_.notify_all();
}

bool DecodeThread::pop(DecodedFrame& out) {
    std::unique_lock<std::mutex> lk(mu_);
    if (size_ == 0 && !eof_ && !stop_.load()) {
        // The consumer is about to wait, which CP 8.1 says should not happen.
        // Counted rather than asserted: the number goes into run.json and a
        // regression shows up as a rising count instead of a vanished claim.
        waits_.fetch_add(1);
    }
    // ----------------------------------------------------------------------
    // THE WATCHDOG — P0-3.
    //
    // This was an unbounded wait. With the decoder single-threaded that was
    // merely optimistic; with worker threads it is the difference between a
    // diagnosable failure and the nine-minute CI hang that made threading get
    // switched off in the first place.
    //
    // wait_for rather than wait, and on a stall the run ends with a message
    // naming the remedy. The timeout is three orders of magnitude above the
    // slowest legitimate frame (a 2000x2000 H.264 frame is 5-10 ms), so it
    // cannot fire on a merely slow machine — the failure it catches is a
    // permanent deadlock, not slowness.
    //
    // It deliberately does not try to kill the decoder thread: there is no
    // portable way to interrupt a thread blocked inside a third-party library,
    // and attempting it would trade a hang for a crash. close() detaches
    // instead of joining when this has fired.
    // ----------------------------------------------------------------------
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(stall_timeout_s_);
    const bool ready = not_empty_.wait_until(
        lk, deadline, [this] { return size_ > 0 || eof_ || stop_.load(); });
    if (!ready) {
        stalled_.store(true);
        return false;
    }
    if (size_ == 0) return false;

    out = std::move(ring_[tail_]);
    tail_ = (tail_ + 1) % ring_.size();
    --size_;
    lk.unlock();
    not_full_.notify_one();
    return true;
}

void DecodeThread::close() {
    stop_.store(true);
    not_full_.notify_all();
    not_empty_.notify_all();

    if (thread_.joinable()) {
        if (stalled_.load()) {
            // ----------------------------------------------------------
            // DETACH, DO NOT JOIN — and leak the Impl deliberately.
            //
            // The watchdog fired, which means the decode thread is blocked
            // inside FFmpeg and is not coming back. Joining it would hang
            // here exactly as it hung in pop(), turning a diagnosable
            // failure into the original nine-minute freeze one function
            // later.
            //
            // The thread still holds a reference to *impl_ and to the ring.
            // Destroying either while it is blocked inside read() is a
            // use-after-free waiting for the decoder to wake up. So the
            // Impl is RELEASED rather than reset: the cv::VideoCapture and
            // its buffers stay allocated for the life of the process, which
            // is a bounded, one-off leak on a path that is already a fatal
            // error and is about to end the run.
            //
            // The ring is likewise left alone.
            //
            // This is the honest cost of a watchdog over a library that
            // offers no cancellation. It is recorded here rather than
            // hidden because a reader finding a deliberate leak deserves
            // the reason.
            // ----------------------------------------------------------
            thread_.detach();
            (void)impl_.release();
            head_ = tail_ = size_ = 0;
            return;
        }
        thread_.join();
    }
    impl_.reset();
    ring_.clear();
    head_ = tail_ = size_ = 0;
}

#else   // !SAT_HAVE_OPENCV

// A build with no video support still compiles and still links. Every video
// path checks video_support_compiled_in() and reports a clear message, which is
// INV-7's principle generalised to an optional dependency.
struct DecodeThread::Impl {};
DecodeThread::DecodeThread() = default;
DecodeThread::~DecodeThread() = default;
Status DecodeThread::open(const std::filesystem::path&, int, int) {
    return Err("this build has no video support (OpenCV was not found at "
               "configure time)");
}
void DecodeThread::run() {}
bool DecodeThread::pop(DecodedFrame&) { return false; }
void DecodeThread::close() {}
int  DecodeThread::default_threads() noexcept { return 1; }

#endif

}  // namespace sat
