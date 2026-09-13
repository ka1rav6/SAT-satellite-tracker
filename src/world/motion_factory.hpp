// world/motion_factory.hpp — see motion_factory.cpp.

#pragma once

#include "scenario/scenario.hpp"
#include "world/motion_component.hpp"

#include <memory>

namespace sat {

/// Build one motion component from its parsed TOML form.
///
/// Returns nullptr for an unrecognised kind. The schema rejects those before
/// this is reached, so a nullptr here means a kind was added to the parser and
/// not to the factory.
[[nodiscard]] std::unique_ptr<IMotionComponent> build_motion_component(const MotionSpec&);

}  // namespace sat
