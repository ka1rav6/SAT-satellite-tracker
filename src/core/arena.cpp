// core/arena.cpp — the allocation-trap flag.
//
// Defined in its own translation unit so that every module sees the same
// object. See INV-4 and CP 14.3.

#include "core/arena.hpp"

namespace sat {

bool g_in_frame = false;

} // namespace sat
