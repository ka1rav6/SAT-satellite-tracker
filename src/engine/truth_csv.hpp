// engine/truth_csv.hpp — CP 8.7.
//
// "--truth CSV ingest and self-scoring. Accept when: feeding a self-generated
//  video with known truth reports the expected error."
//
// ---------------------------------------------------------------------------
// WHY SELF-SCORING IS THE ONLY HONEST WAY TO TRUST A VIDEO NUMBER
// ---------------------------------------------------------------------------
// In synthetic mode the simulator knows where the beacon is, so a centroiding
// error is exact by construction. In video mode it knows nothing: the clip is
// somebody else's, and without truth the system can report where it THINKS the
// beacon is and no more.
//
// That is a problem for the 30%-weighted benchmark, because it means the video
// path is the part of the system least able to prove it works. The answer is to
// close the loop on ourselves first: render a video from a scenario whose truth
// we already have, write that truth to a CSV, feed the video back in as if it
// came from outside, and check the reported centroids against it. If the
// numbers come back at synthetic-mode accuracy, the video path is not
// introducing error; if they do not, the difference is exactly what the crop,
// the codec and the container cost.
//
// The format is deliberately the same columns as §13.2's centroid.csv, minus
// the ones only a tracker produces. A truth file and an output file are then
// directly comparable, and a truth file can be produced by a run of this
// program — which is what makes the self-scoring loop a single command rather
// than a script.

#pragma once

#include "core/result.hpp"
#include "engine/frame_source.hpp"

#include <filesystem>
#include <vector>

namespace sat {

/// Parse a truth CSV into one FrameTruth per frame, indexed by frame number.
///
/// Expected columns, with a header line naming them:
///     frame, time_s, cx_screen, cy_screen
/// Extra columns are ignored, so a centroid.csv from a previous run can be fed
/// straight back in as truth. Missing frames leave gaps, which are reported as
/// "no truth this frame" rather than interpolated — INV-9's principle applies
/// to reference data as much as to measurements.
[[nodiscard]] Result<std::vector<FrameTruth>>
load_truth_csv(const std::filesystem::path& path, const ScreenGeometry& screen);

/// Parse from memory. Used by the tests, which should not need a filesystem.
[[nodiscard]] Result<std::vector<FrameTruth>>
parse_truth_csv(std::string_view text, const ScreenGeometry& screen,
                std::string_view name = "<memory>");

}  // namespace sat
