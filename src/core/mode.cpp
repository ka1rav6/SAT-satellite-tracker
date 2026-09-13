// core/mode.cpp

#include "core/mode.hpp"

namespace sat {

const char* track_mode_name(TrackMode m) noexcept {
    switch (m) {
        case TrackMode::Idle:      return "IDLE";
        case TrackMode::Search:    return "SEARCH";
        case TrackMode::Detect:    return "DETECT";
        case TrackMode::Acquire:   return "ACQUIRE";
        case TrackMode::Track:     return "TRACK";
        case TrackMode::Reacquire: return "REACQUIRE";
        case TrackMode::Handover:  return "HANDOVER";
        case TrackMode::Safe:      return "SAFE";
    }
    return "UNKNOWN";
}

}  // namespace sat
