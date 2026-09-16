// core/strategy.hpp — the vocabulary the SAT supervisor speaks (design §10.6).
//
// ---------------------------------------------------------------------------
// WHY THESE ENUMS ARE HERE AND NOT WHERE EACH ONE IS USED
// ---------------------------------------------------------------------------
// Exactly the argument core/mode.hpp makes for TrackMode, arriving a second
// time for the same structural reason.
//
// §10.6's Strategy names which detector, which filter and which predictor to
// run. The supervisor PRODUCES those values and lives in sat_control; the
// engine CONSUMES them. sat_control cannot include engine/pipeline.hpp —
// sat_engine links sat_world, and cmake/modules.cmake fails the configure
// outright if ground truth enters the controller's dependency closure (INV-1).
//
// So the detector selector could not stay a nested enum inside PipelineConfig,
// where it started. A plain enum with no dependencies belongs at the bottom of
// the graph, where everything can see it and it can see nothing.
//
// CentroidKind and SearchStrategy are NOT duplicated here. They already live in
// perception/ and search/, which sat_control reaches legitimately, and a second
// declaration of either would be the parallel-table drift that
// scenario/overlay.hpp spends twenty lines warning about.

#pragma once

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// DetectorKind — §10.6's PerceptionKind.
//
// Was PipelineConfig::Detector. The name changed with the move because
// "Detector" alone is ambiguous at namespace scope — the project also has a
// QuadrantDetector — and because §10.6 calls the field a KIND.
// ---------------------------------------------------------------------------
enum class DetectorKind : uint8_t {
    /// §9.4's pipeline. The default, and §9.4 is explicit that the straw man
    /// "must never be the default".
    Classical = 0,
    /// The Stage 1 straw man. Kept selectable because CP 4.11 grades a real
    /// result with it — "with 120 clutter sources, the brightest-pixel detector
    /// demonstrably locks onto the wrong thing" — and that ablation needs it
    /// driving the whole closed loop, not just a unit test.
    BrightestPixel,
    /// Stage 11's CandidateNet in the loop. Falls back to Classical until it
    /// exists, and under --no-ai, which is INV-7.
    MlAssisted,
};

[[nodiscard]] const char* detector_kind_name(DetectorKind k) noexcept;

// ---------------------------------------------------------------------------
// FilterKind — which motion estimator runs.
//
// Both of these are real as of CP 10.5. Cv is tracking/kalman.hpp's four-state
// constant-velocity filter; Imm is tracking/imm.hpp's six-state CV/CA/CT bank.
// ---------------------------------------------------------------------------
enum class FilterKind : uint8_t { Cv = 0, Imm };

[[nodiscard]] const char* filter_kind_name(FilterKind k) noexcept;

// ---------------------------------------------------------------------------
// PredictorKind — delay compensation.
//
// Smith is control/smith.hpp, and CP 10.4 measured it as a net LOSS on this
// plant: the loop is not delay-limited, 20 degrees of phase at crossover out
// of a margin near 90. It is in the supervisor's vocabulary anyway, because
// the supervisor's job is to pick per conditions and the conditions under
// which a predictor helps are exactly the ones CP 10.4's threshold test
// describes. Today the rule table never selects it, and says why.
// ---------------------------------------------------------------------------
enum class PredictorKind : uint8_t { None = 0, Smith };

[[nodiscard]] const char* predictor_kind_name(PredictorKind k) noexcept;

}  // namespace sat
