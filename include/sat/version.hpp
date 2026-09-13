// include/sat/version.hpp — build identity.
//
// CMake injects SAT_VERSION from project(VERSION) and SAT_GIT_HASH from
// `git rev-parse`. Both end up in every centroid.csv header and every run.json
// (design §13.2), because a benchmark number that cannot be traced back to a
// commit is not evidence of anything.
//
// The fallbacks keep editors and one-off compiles working without configuring.

#pragma once

#ifndef SAT_VERSION
#define SAT_VERSION "0.1.0"
#endif

// Short commit hash of the build, or "unknown" outside a git checkout.
#ifndef SAT_GIT_HASH
#define SAT_GIT_HASH "unknown"
#endif

// Human-readable product name used in CLI banners and log headers.
#ifndef SAT_PRODUCT_NAME
#define SAT_PRODUCT_NAME "SAT — Satellite Adaptive Tracker"
#endif
