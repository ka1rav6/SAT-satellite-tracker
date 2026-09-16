// engine/events.hpp — design §7.4's timeline, actually executed.
//
// ---------------------------------------------------------------------------
// THIS WAS CONFIGURATION THAT DID NOTHING
// ---------------------------------------------------------------------------
// §7.4 specifies four actions and gives them a TOML syntax. The loader parsed
// them, the schema validated them — it checks that t_s is non-negative and
// inside the run's duration, and rejects an unknown action — and run.json
// echoed them back. Nothing read them. A scenario could ask for fog at t = 8 s,
// be told the request was valid, and run in clear air for the whole run.
//
// That is the worst shape a defect can take in a configuration system: the
// validation says yes, the artifact says the setting was applied, and the
// behaviour is unchanged. It was found at Stage 12, looking for a scenario
// whose CONDITIONS change — because a supervisor that adapts to conditions
// cannot be demonstrated, or refuted, on a scenario where they never do.
//
// ---------------------------------------------------------------------------
// WHY THE SCHEDULER IS ITS OWN FILE
// ---------------------------------------------------------------------------
// Events reach across the whole simulator: atmosphere lives in the sensor
// chain, occlusion in the emitter array, decoys in the world, gusts in the
// disturbance model. Putting the dispatch inline in Pipeline::step would put
// four unrelated pokes into the hot path and make "did this run have events"
// unanswerable without reading it.
//
// Here it is one object with one question — "what is due by time t" — and the
// engine applies what comes back. That also makes the ORDERING explicit, which
// matters: two events at the same timestamp must fire in the order the file
// lists them, or a scenario is not reproducible from its own text.

#pragma once

#include "scenario/scenario.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace sat {

/// §7.4's validated action enum, resolved once at build time so the hot path
/// never compares strings.
enum class EventAction : uint8_t {
    SetAtmosphere = 0,
    OccludeTarget,
    SpawnDecoy,
    PlatformGust,
    Unknown,
};

[[nodiscard]] EventAction parse_event_action(std::string_view s) noexcept;
[[nodiscard]] const char* event_action_name(EventAction a) noexcept;

/// One scheduled event, resolved from an EventSpec.
struct ScheduledEvent {
    double      t_s = 0.0;
    EventAction action = EventAction::Unknown;
    Atmosphere  atmosphere = Atmosphere::Clear;
    double      ramp_s = 0.0;
    double      duration_s = 0.0;
    double      offset_px[2] = {0.0, 0.0};
    double      magnitude_px = 0.0;
    bool        fired = false;
};

// ---------------------------------------------------------------------------
// EventTimeline — what is due, and what is currently in force.
//
// Two different things, and keeping them apart is the point. `occlude_target`
// and `platform_gust` have a DURATION: they are not instants, they are
// intervals, and something has to remember to undo them. An implementation
// that only fires edges leaves the target occluded for the rest of the run,
// which is the obvious bug and the one worth designing out rather than fixing.
// ---------------------------------------------------------------------------
class EventTimeline {
public:
    /// Resolve a scenario's events. Sorted by time with ties broken by file
    /// order, so a scenario is reproducible from its own text.
    void build(const Scenario& sc);

    /// Advance to `t_s` and return the events that became due since the last
    /// call. Edge-triggered: each fires exactly once.
    ///
    /// The span points into this object and is valid until the next call.
    [[nodiscard]] std::span<const ScheduledEvent*> due(double t_s);

    /// True while an `occlude_target` interval covers `t_s`.
    [[nodiscard]] bool target_occluded(double t_s) const noexcept;

    // -----------------------------------------------------------------------
    // Does this timeline contain ANY occlusion event at all.
    //
    // The engine asks before it touches the target's brightness, and the
    // reason is a defect this caused. Restoring the configured intensity every
    // frame — "not occluded, so put it back" — means the timeline OWNS that
    // field for the whole run, and silently overwrites anything else that sets
    // it. tests/tracking/test_reacquire.cpp blanks the beacon by zeroing
    // exactly that field, and CP 6.7's dropout cases stopped seeing a dropout:
    // the timeline handed the beacon back every frame.
    //
    // A component must not write what it does not own. With no occlusion
    // events there is nothing to restore, so the field is left alone.
    // -----------------------------------------------------------------------
    [[nodiscard]] bool has_occlusions() const noexcept { return has_occlusions_; }

    /// The gust offset in force at `t_s`, screen pixels. Zero outside any gust.
    [[nodiscard]] void gust_offset(double t_s, double& dx, double& dy) const noexcept;

    [[nodiscard]] size_t count() const noexcept { return events_.size(); }
    [[nodiscard]] const std::vector<ScheduledEvent>& events() const noexcept {
        return events_;
    }

    /// Re-arm without re-parsing, for a repeated run of the same scenario.
    void rewind() noexcept;

private:
    std::vector<ScheduledEvent>  events_;
    std::vector<const ScheduledEvent*> due_;   ///< scratch, reserved at build
    size_t next_ = 0;
    bool   has_occlusions_ = false;
};

}  // namespace sat
