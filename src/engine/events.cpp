// engine/events.cpp — design §7.4.

#include "engine/events.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

EventAction parse_event_action(std::string_view s) noexcept {
    if (s == "set_atmosphere") return EventAction::SetAtmosphere;
    if (s == "occlude_target") return EventAction::OccludeTarget;
    if (s == "spawn_decoy")    return EventAction::SpawnDecoy;
    if (s == "platform_gust")  return EventAction::PlatformGust;
    return EventAction::Unknown;
}

const char* event_action_name(EventAction a) noexcept {
    switch (a) {
        case EventAction::SetAtmosphere: return "set_atmosphere";
        case EventAction::OccludeTarget: return "occlude_target";
        case EventAction::SpawnDecoy:    return "spawn_decoy";
        case EventAction::PlatformGust:  return "platform_gust";
        case EventAction::Unknown:       return "unknown";
    }
    return "unknown";
}

namespace {

Atmosphere parse_atmosphere(std::string_view s) noexcept {
    if (s == "fog")  return Atmosphere::Fog;
    if (s == "haze") return Atmosphere::Haze;
    if (s == "rain") return Atmosphere::Rain;
    return Atmosphere::Clear;
}

}  // namespace

void EventTimeline::build(const Scenario& sc) {
    events_.clear();
    events_.reserve(sc.events.size());
    for (const EventSpec& e : sc.events) {
        ScheduledEvent s;
        s.t_s          = e.t_s;
        s.action       = parse_event_action(e.action);
        s.atmosphere   = parse_atmosphere(e.mode);
        s.ramp_s       = e.ramp_s;
        s.duration_s   = e.duration_s;
        s.offset_px[0] = e.offset_px[0];
        s.offset_px[1] = e.offset_px[1];
        s.magnitude_px = e.magnitude_px;
        events_.push_back(s);
    }
    // STABLE sort: §7.4 lets two events share a timestamp, and which fires
    // first is then decided by file order. std::sort would be free to swap
    // them and the scenario would stop being reproducible from its own text —
    // which is INV-3's whole point, arriving somewhere unexpected.
    std::stable_sort(events_.begin(), events_.end(),
                     [](const ScheduledEvent& a, const ScheduledEvent& b) {
                         return a.t_s < b.t_s;
                     });
    has_occlusions_ = false;
    for (const ScheduledEvent& e : events_) {
        if (e.action == EventAction::OccludeTarget) { has_occlusions_ = true; break; }
    }
    due_.clear();
    due_.reserve(events_.size());
    next_ = 0;
}

void EventTimeline::rewind() noexcept {
    for (ScheduledEvent& e : events_) e.fired = false;
    next_ = 0;
}

std::span<const ScheduledEvent*> EventTimeline::due(double t_s) {
    due_.clear();
    // Sorted, so everything due is a prefix from next_. A scan over all events
    // every frame would also be correct and would be O(n) per frame for an n
    // that a scenario controls.
    while (next_ < events_.size() && events_[next_].t_s <= t_s) {
        events_[next_].fired = true;
        due_.push_back(&events_[next_]);
        ++next_;
    }
    return std::span<const ScheduledEvent*>(due_.data(), due_.size());
}

bool EventTimeline::target_occluded(double t_s) const noexcept {
    for (const ScheduledEvent& e : events_) {
        if (e.action != EventAction::OccludeTarget) continue;
        if (t_s >= e.t_s && t_s < e.t_s + e.duration_s) return true;
    }
    return false;
}

void EventTimeline::gust_offset(double t_s, double& dx, double& dy) const noexcept {
    dx = 0.0;
    dy = 0.0;
    for (const ScheduledEvent& e : events_) {
        if (e.action != EventAction::PlatformGust) continue;
        if (t_s < e.t_s || t_s >= e.t_s + e.duration_s) continue;
        // -------------------------------------------------------------------
        // A gust is a smooth excursion and back, not a step.
        //
        // A step would be indistinguishable from a commanded slew as far as the
        // loop is concerned, and it would also be unphysical: a platform on a
        // vehicle or a mast does not teleport. A raised-cosine over the
        // duration starts and ends at zero WITH ZERO SLOPE, so nothing in the
        // loop sees a discontinuity in either position or rate — which matters,
        // because the blur model differentiates the boresight and a rate step
        // would smear one frame by an amount the specification never asked for.
        //
        // Direction is the diagonal, so both axes are exercised. §7.4 gives a
        // magnitude and no direction; picking one axis would silently make the
        // event a test of azimuth only.
        // -------------------------------------------------------------------
        const double u = (t_s - e.t_s) / (e.duration_s > 0.0 ? e.duration_s : 1.0);
        const double w = 0.5 * (1.0 - std::cos(2.0 * 3.14159265358979 * u));
        const double a = e.magnitude_px * w / std::sqrt(2.0);
        dx += a;
        dy += a;
    }
}

}  // namespace sat
