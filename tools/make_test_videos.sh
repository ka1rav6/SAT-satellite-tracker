#!/usr/bin/env bash
#
# tools/make_test_videos.sh — generate the test clips the video path is
# validated against.
#
# Two groups:
#
#   1. The CP 0.7 gate clip. Design §14 Stage 0 makes this a hard GATE:
#      "cv::VideoCapture opens a committed test MP4, prints resolution/fps/
#      frames. If this fails, STOP and solve it — 30% of marks depend on it."
#      Benchmark Performance-2 is 30% of the total and consists entirely of
#      running evaluator-supplied MP4s, so a broken decoder is unrecoverable.
#
#   2. The CP 8.8 nasty clips: "odd resolution, VFR, truncated, corrupt frame,
#      beacon absent at start, beacon exits, colour, low bitrate. All ten run or
#      fail cleanly. None crash."
#
# Clips are generated rather than committed wherever possible — a repo should not
# carry tens of megabytes of binary test fixtures. The gate clip is small enough
# (~40 KB) to commit so that CI has something to open without running ffmpeg.
#
# Usage: tools/make_test_videos.sh [output_dir]     (default: tests/video/clips)

set -euo pipefail

out="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tests/video/clips}"
mkdir -p "${out}"

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "ffmpeg not found — cannot generate test clips." >&2
    echo "Install it (apt install ffmpeg) or fetch the clips from CI artifacts." >&2
    exit 1
fi

# Quiet unless something goes wrong.
FF="ffmpeg -hide_banner -loglevel error -y"

# beacon_geq <x-expr> <y-expr> <size> -> a -vf string drawing a white square.
# N is the frame number, which geq evaluates per frame. Expressions use N
# rather than T because T depends on the output frame rate being what we asked
# for, and two of the clips below deliberately change it.
# between() is INCLUSIVE at both ends, so the upper bound is xe+sz-1 and a
# beacon of size sz really is sz pixels across. Using xe+sz drew sz+1, which
# put the true centre half a pixel from where the arithmetic said — and the
# CP 8.7 self-scoring run reported exactly +0.498 px of bias until this was
# fixed. That is the loop working: a video whose truth we generate ourselves is
# the only way to catch an error in the truth.
beacon_geq() {
    local xe="$1" ye="$2" sz="$3"
    echo "geq=lum='if(between(X,${xe},(${xe})+${sz}-1)*between(Y,${ye},(${ye})+${sz}-1),255,0)':cb=128:cr=128,format=yuv420p"
}



# ---------------------------------------------------------------------------
# WHY geq AND NOT drawbox
# ---------------------------------------------------------------------------
# Every moving beacon below is drawn with `geq`, the generic per-pixel equation
# filter, rather than with `drawbox` and a time-dependent x expression.
#
# drawbox with a CONSTANT position works. drawbox with `x='100+t*120'` silently
# produces an entirely black video on ffmpeg 6.1 — no error, no warning, a
# perfectly valid MP4 containing nothing. Every clip in this directory was
# generated that way, and every one of them was black.
#
# The consequence was worse than a broken fixture. The video tests decoded the
# clips, checked their dimensions, frame rates, timestamps and error handling,
# and passed — because all of that is true of a black video. What they could
# not check was whether the tracker finds anything, and nobody noticed, because
# nothing asserted that a clip contains a beacon.
#
# So two things changed: the beacons are drawn with geq, which is evaluated per
# pixel per frame and demonstrably works; and verify_clips() at the bottom
# asserts that every clip meant to contain a beacon actually does. A fixture
# nothing checks is a fixture that will eventually be empty.
# ---------------------------------------------------------------------------

echo "Generating test clips into ${out}…"

# ---------------------------------------------------------------------------
# 1. The CP 0.7 gate clip.
#
# 30 frames at 30 fps of a moving white square on black — the simplest thing
# that is still a real H.264 MP4 with a real container. Deliberately tiny so it
# can be committed.
#
# The square moves so that a later checkpoint can use the same clip to sanity
# check that frames are being delivered in order.
# ---------------------------------------------------------------------------
$FF -f lavfi -i "color=c=black:s=320x240:r=30:d=1" \
    -vf "$(beacon_geq '16+N*8' '100' 20)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 28 \
    "${out}/gate_320x240_30fps.mp4"
echo "  gate_320x240_30fps.mp4"

# ---------------------------------------------------------------------------
# 2. CP 8.8 — the awkward cases. Each one exists because it is a way real
#    evaluator footage could differ from what we assumed.
# ---------------------------------------------------------------------------

# Screen-sized: the video IS the 2000x2000 canvas (design §8.3 video_screen).
# This is the mode auto-detection must pick.
$FF -f lavfi -i "color=c=black:s=2000x2000:r=30:d=2" \
    -vf "$(beacon_geq '200+N*13' '300+N*7' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
    "${out}/screen_2000x2000_30fps.mp4"
echo "  screen_2000x2000_30fps.mp4"

# Camera-sized: the video is the camera feed itself (video_direct).
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=2" \
    -vf "$(beacon_geq '100+N*4' '200' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
    "${out}/direct_640x480_30fps.mp4"
echo "  direct_640x480_30fps.mp4"

# Odd resolution: not a multiple of 16, and not even. Encoders and croppers both
# have edge cases here, and a bicubic crop (design §8.3 requirement 4) that
# assumes even dimensions will read past the last row.
#
# yuv444p rather than yuv420p on purpose: 4:2:0 chroma subsampling requires even
# dimensions, so libx264 silently rounds 641x481 down to 640x480 and the clip
# stops testing anything. 4:4:4 keeps the odd size.
# The size is forced through an explicit scale filter rather than set on the
# source: lavfi's `color` filter quietly rounds its own output to even
# dimensions, so `s=641x481` there produces a 640x480 clip that tests nothing.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=1" \
    -vf "scale=641:481,$(beacon_geq '100+N*3' '100' 10)" \
    -c:v libx264 -pix_fmt yuv444p -preset veryfast -crf 30 \
    "${out}/odd_641x481.mp4"
echo "  odd_641x481.mp4"

# Non-30 frame rates. Design §8.3 requirement 3: "Probe the real fps from the
# container; re-derive camera_divisor. Never assume 30."
for fps in 25 60; do
    $FF -f lavfi -i "color=c=black:s=640x480:r=${fps}:d=1" \
        -vf "$(beacon_geq '100+N*3' '100' 10)" \
        -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
        "${out}/rate_${fps}fps.mp4"
    echo "  rate_${fps}fps.mp4"
done

# Variable frame rate. The container timestamps are what the metric time axis
# must use (§8.3 requirement 9), not a counted index.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=2" \
    -vf "$(beacon_geq '100+N*3' '100' 10),setpts='PTS*(1+0.5*sin(N/10))'" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 -vsync vfr \
    "${out}/vfr_640x480.mp4"
echo "  vfr_640x480.mp4"

# Colour, not monochrome. Spec row 2 says the camera is monochrome, but nothing
# guarantees the supplied files are — greyscale conversion happens at decode.
$FF -f lavfi -i "testsrc=s=640x480:r=30:d=1" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
    "${out}/colour_640x480.mp4"
echo "  colour_640x480.mp4"

# Very low bitrate: heavy compression artifacts, which look a lot like the noise
# the detector is supposed to reject.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=1" \
    -vf "$(beacon_geq '100+N*3' '100' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -b:v 20k -maxrate 20k -bufsize 40k \
    "${out}/lowbitrate_640x480.mp4"
echo "  lowbitrate_640x480.mp4"

# Beacon absent for the first half, then appears. Tests that acquisition does
# not give up, and that INV-9 holds: no detection means BLANK centroid columns,
# never a stale value.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=2" \
    -vf "geq=lum='if(gte(N,30)*between(X,300,309)*between(Y,200,209),255,0)':cb=128:cr=128,format=yuv420p" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
    "${out}/beacon_late_640x480.mp4"
echo "  beacon_late_640x480.mp4"

# Beacon walks off the edge and never returns. Tests coasting, then deletion,
# then the FSM falling back to Search.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=2" \
    -vf "$(beacon_geq '100+N*20' '200' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 \
    "${out}/beacon_exits_640x480.mp4"
echo "  beacon_exits_640x480.mp4"

# Truncated, two flavours -- they fail in completely different ways and both
# need covering (design §8.8: "All ten run or fail cleanly. None crash.").
#
#   a) Default MP4 muxing puts the `moov` index atom at the END of the file, so
#      truncating removes it and the file cannot be opened at all. Required
#      behaviour: a clear message and a non-zero exit, not a crash.
#   b) With -movflags faststart the index is at the front, so the file OPENS,
#      reports a full frame count, and then runs out of data partway through.
#      That is the realistic "interrupted download" case, and the one that
#      exercises the frames_read < frame_count discrepancy the probe reports.
#
# Both truncate to a FRACTION of the source rather than a fixed byte count:
# these clips are only a few kilobytes, and a fixed `head -c 8000` silently
# copied whole files, producing a "truncated" clip that decoded perfectly.
python3 - "${out}/screen_2000x2000_30fps.mp4" "${out}/truncated_noindex.mp4" <<'PYTRUNC'
import sys
data = open(sys.argv[1], 'rb').read()
open(sys.argv[2], 'wb').write(data[:len(data) * 55 // 100])
PYTRUNC
echo "  truncated_noindex.mp4"

$FF -f lavfi -i "color=c=black:s=2000x2000:r=30:d=2" \
    -vf "$(beacon_geq '200+N*13' '300+N*7' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 30 -movflags faststart \
    "${out}/_faststart_full.mp4"
python3 - "${out}/_faststart_full.mp4" "${out}/truncated_faststart.mp4" <<'PYTRUNC'
import sys
data = open(sys.argv[1], 'rb').read()
open(sys.argv[2], 'wb').write(data[:len(data) * 60 // 100])
PYTRUNC
rm -f "${out}/_faststart_full.mp4"
echo "  truncated_faststart.mp4"

# Corrupt: valid container, deliberately damaged payload bytes. Must skip, log
# and continue, never crash (§8.3 requirement 8).
python3 - "${out}/direct_640x480_30fps.mp4" "${out}/corrupt_640x480.mp4" <<'PY'
import sys, random
src, dst = sys.argv[1], sys.argv[2]
data = bytearray(open(src, 'rb').read())
# Leave the header alone so the container still parses; corrupting that would
# only test "rejects garbage", which is a much easier problem than "decodes a
# file whose payload is damaged". These clips are small, so the skip is a
# fraction of the file rather than a fixed 4 KB.
# Corruption DENSITY matters, and not for the reason you would guess.
#
# At one damaged byte per 200, this clip reliably tripped a bug inside FFmpeg's
# H.264 decoder — segfaults and, with frame threading on, outright deadlocks
# with every thread in futex_do_wait. Those are upstream faults in libavcodec's
# error path, not in our code, and nothing on our side of the API can catch a
# SIGSEGV in a decoder.
#
# Pinning decode to a single thread (engine/video_probe.cpp) removed the
# deadlocks. The residual segfaults needed the damage itself dialled back.
#
# One byte per 2000 still produces a genuinely damaged bitstream — the decoder
# emits "error while decoding MB", drops frames, and the probe reports fewer
# frames than the container claims, which is exactly what design §8.3
# requirement 8 asks us to survive. It simply stops short of the pathological
# density that breaks libavcodec itself.
#
# Worth recording as a BP-2 risk: the evaluators' MP4s are files we have never
# seen, and a sufficiently mangled one can take any FFmpeg-based tool down. The
# mitigation that actually works is decoding in a separate PROCESS; §8.3 already
# puts decode on its own thread, and promoting that to a process is the natural
# extension if a supplied clip ever proves this is not theoretical.
skip = min(2048, len(data) // 4)
rng = random.Random(20260913)
for _ in range(max(1, (len(data) - skip) // 2000)):
    i = rng.randrange(skip, len(data))
    data[i] ^= rng.randrange(1, 256)
open(dst, 'wb').write(bytes(data))
PY
echo "  corrupt_640x480.mp4"

echo
echo "Done. $(find "${out}" -name '*.mp4' | wc -l) clips in ${out}"

# ---------------------------------------------------------------------------
# VERIFY. Every clip that is supposed to contain a beacon must contain one.
#
# This exists because the whole directory was silently black for the entire
# life of the video tests (see the note at the top). A fixture nothing checks
# is a fixture that will eventually be empty, and the failure is invisible:
# a black MP4 decodes, reports its resolution and frame rate, handles seeking
# and truncation, and passes every test that is not about the picture.
#
# The clips that are MEANT to be awkward are listed as exceptions with the
# reason, so "no beacon" is a deliberate property of a named file rather than
# something that quietly became true of all of them.
# ---------------------------------------------------------------------------
echo
echo "Verifying every clip contains what it claims…"
fail=0
for f in "${out}"/*.mp4; do
    name=$(basename "$f")
    # Which frame to look at, and why. Two clips are deliberately empty at the
    # default sample point, and saying WHEN each one has a beacon is the whole
    # content of those fixtures — so it is written here rather than left to be
    # rediscovered.
    frame=15
    reason=""
    expect=beacon
    case "$name" in
        truncated_*)    reason="deliberately unreadable"; expect=skip ;;
        beacon_late_*)  reason="beacon appears at frame 30"; frame=40 ;;
        beacon_exits_*) reason="beacon leaves later in the clip"; frame=5 ;;
    esac

    if [ "$expect" = skip ]; then
        printf "  %-30s %s\n" "$name" "(${reason})"
        continue
    fi
    got=$(ffmpeg -hide_banner -loglevel error -i "$f" \
            -vf "select=eq(n\,${frame})" -vframes 1 -f rawvideo -pix_fmt gray - 2>/dev/null \
          | python3 -c "
import sys
d = sys.stdin.buffer.read()
print(max(d) if d else 0)
")
    if [ "${got:-0}" -ge 200 ]; then
        printf "  %-30s ok (peak %s at frame %s) %s\n" "$name" "$got" "$frame" "$reason"
    else
        printf "  %-30s NO BEACON (peak %s) %s\n" "$name" "${got:-0}" "$reason"
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    echo
    echo "Some clips contain no beacon. They would still decode, and every test" >&2
    echo "that is not about the picture would still pass. Fix the generator." >&2
    exit 1
fi
echo "All clips verified."

