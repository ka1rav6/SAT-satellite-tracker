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
# 3. THE BP-2 BENCHMARK FIXTURES — P0-4.
# ---------------------------------------------------------------------------
# Benchmark Performance-2 is 30 % of the total marks and consists entirely of
# running evaluator-supplied MP4s, scored on "Comparison of Centroiding error
# with predefined error values". Until these existed the project had NO
# end-to-end number for it at all:
#
#   * `just video <file>` did not pass --truth, so nothing was scored;
#   * no truth CSV was committed for any clip;
#   * the CP 8.7 self-scoring test deliberately used "a plain
#     intensity-weighted centroid rather than the perception pipeline", ran the
#     SOURCE alone rather than the closed loop, and only in direct mode.
#
# So the project could say "the decoder works" and "the perception pipeline
# works" and had never demonstrated clip in -> full pipeline -> centroid.csv ->
# RMSE against truth.
#
# WHY THE EXISTING CLIPS CANNOT SERVE. direct_640x480_30fps.mp4 is a NOISELESS
# white box on black, and the full pipeline scores 0.0003 px RMSE on it. A
# perfectly symmetric noiseless box has an exact centroid; measuring it
# measures nothing. screen_2000x2000_30fps.mp4 is likewise noiseless, and its
# beacon moves at 13 px/frame across the screen — faster than a 5 deg/s mount
# can follow — so the loop never achieves a sustained lock and the run reports
# no centroiding frames.
#
# The three below are built to be benchmarks rather than decoder fixtures:
# noisy, at a beacon speed the mount can actually track, and with truth that is
# exact BY CONSTRUCTION rather than measured.
#
# ---------------------------------------------------------------------------
# TRUTH IS GENERATED FROM THE SAME EXPRESSION THAT DRAWS THE BEACON
# ---------------------------------------------------------------------------
# The beacon's top-left corner at frame N is (x0 + vx*N, y0 + vy*N) and it is
# `sz` pixels across, so its centre is at
#
#     (x0 + vx*N + (sz-1)/2,  y0 + vy*N + (sz-1)/2)
#
# The (sz-1)/2 term is the one that is easy to get wrong: between() in geq is
# INCLUSIVE at both ends, so a beacon spanning [x, x+sz-1] has its centre half
# a pixel below x + sz/2. The existing beacon_geq() comment records that this
# cost a spurious +0.498 px of measured bias before it was noticed. The truth
# writer below uses the identical arithmetic, so a future change to one is a
# visible inconsistency with the other rather than a silent bias.
#
# Truth is written in SCREEN coordinates, which is what --truth expects and
# what centroid.csv reports, so a truth file and an output file are directly
# comparable column for column.
# ---------------------------------------------------------------------------

# bp2_truth <csv> <frames> <fps> <x0> <y0> <vx> <vy> <size>
bp2_truth() {
    local csv="$1" frames="$2" fps="$3" x0="$4" y0="$5" vx="$6" vy="$7" sz="$8"
    python3 - "$csv" "$frames" "$fps" "$x0" "$y0" "$vx" "$vy" "$sz" <<'PYTRUTH'
import sys
csv, frames, fps, x0, y0, vx, vy, sz = sys.argv[1:9]
frames, fps = int(frames), float(fps)
x0, y0, vx, vy, sz = float(x0), float(y0), float(vx), float(vy), int(sz)
# The centre of a square spanning [x, x+sz-1] inclusive. See the note above.
half = (sz - 1) / 2.0
with open(csv, "w") as f:
    f.write("frame,time_s,cx_screen,cy_screen\n")
    for n in range(frames):
        f.write("%d,%.6f,%.4f,%.4f\n"
                % (n, n / fps, x0 + vx * n + half, y0 + vy * n + half))
PYTRUTH
}

echo
echo "BP-2 benchmark fixtures…"

# (a) NOISELESS DIRECT. Kept, and kept labelled: it is the control. A run that
#     does not score ~0 px here has a bug in the crop or the truth, not in the
#     centroider, and knowing that before looking at the noisy cases saves an
#     afternoon. 4 px/frame is well inside what the mount can follow.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=4" \
    -vf "$(beacon_geq '120+N*4' '180+N*2' 10)" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 18 \
    "${out}/bp2_clean_direct_640x480.mp4"
bp2_truth "${out}/bp2_clean_direct_640x480.csv" 120 30 120 180 4 2 10
echo "  bp2_clean_direct_640x480.mp4 + .csv"

# (b) NOISY DIRECT. Additive temporal noise at spec row 22's cap of 20 grey
#     levels, plus real H.264 compression artefacts. This is the one that says
#     what the centroider does on a picture it did not render, with the crop
#     taken out of the question — it is 640x480 in and 640x480 out.
#
#     CRF 32 for the same size reason as the screen clip: temporal noise is
#     near-incompressible, and at CRF 18 this is 12 MB. Accuracy is flat across
#     the range (0.0014 px at CRF 28, 0.0009 at 30, 0.0014 at 32), so the
#     choice is made on size.
$FF -f lavfi -i "color=c=black:s=640x480:r=30:d=4" \
    -vf "$(beacon_geq '120+N*4' '180+N*2' 10),noise=alls=20:allf=t" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 32 \
    "${out}/bp2_noisy_direct_640x480.mp4"
bp2_truth "${out}/bp2_noisy_direct_640x480.csv" 120 30 120 180 4 2 10
echo "  bp2_noisy_direct_640x480.mp4 + .csv"

# (c) THE FULL BP-2 REHEARSAL: 2000x2000 screen mode, noisy, with the PTZ loop
#     engaged. This is the shape the problem statement describes — "a complete
#     screen with noise and a moving beacon spot" — and it is the only one of
#     the three that exercises acquisition, tracking, handover and the crop
#     together.
#
#     3 px/frame and 1.5 px/frame: 90 px/s and 45 px/s on the screen, against a
#     5 deg/s mount that can slew 800 px/s. Comfortably followable, which is
#     the point — a benchmark the mount cannot physically track measures the
#     mount's rate limit and calls it a centroiding error.
#
#     The beacon starts near the screen centre because spec row 6 puts the
#     camera there at t=0, so it is in the field from the first frame and the
#     run measures tracking rather than a cold search (design 10.5's bound).
#
#     THE ENCODER SETTINGS ARE A MEASUREMENT, NOT A GUESS. Full-frame temporal
#     noise over four megapixels is close to incompressible, so the obvious
#     "noise=alls=18, CRF 20" produces a 144 MB file — uncommittable, and its
#     decode alone costs 24.6 ms/frame, which is most of a 33.3 ms real-time
#     budget spent on an artefact of how the fixture was made rather than on
#     anything the system does. Swept:
#
#       crf  noise     size    image RMSE   tracking (steady)
#        24     12      18M      0.1003 px       0.885 px
#        26     12     7.1M      0.1033 px       0.890 px
#        24      8     2.3M      0.1011 px       0.890 px
#        26      8     176K      0.1037 px       0.888 px   <- chosen
#        22      6     2.1M      0.1000 px       0.886 px
#        28     12     644K      0.1107 px      88.080 px   <- loses lock
#
#     Accuracy is flat across the whole usable range, so the choice is made on
#     size. CRF 28 is past the cliff: the blocking artefacts it introduces on a
#     2000x2000 near-black field feed the CFAR stage enough false candidates to
#     cost the lock, which is a real and interesting failure but not what this
#     fixture is for.
$FF -f lavfi -i "color=c=black:s=2000x2000:r=30:d=6" \
    -vf "$(beacon_geq '995+N*3' '995+N*1.5' 10),noise=alls=8:allf=t" \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 26 \
    "${out}/bp2_screen_2000x2000.mp4"
bp2_truth "${out}/bp2_screen_2000x2000.csv" 180 30 995 995 3 1.5 10
echo "  bp2_screen_2000x2000.mp4 + .csv"

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

