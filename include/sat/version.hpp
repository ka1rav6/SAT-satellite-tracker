// include/sat/version.hpp — build identity.
//
// CMake injects SAT_VERSION from project(VERSION) as a compile definition, and
// SAT_GIT_HASH through the generated header included below. Both end up in
// every centroid.csv header and every run.json (design §13.2), because a
// benchmark number that cannot be traced back to a commit is not evidence of
// anything.
//
// The two arrive by different routes on purpose. The version changes only when
// CMakeLists.txt does, and that file is a configure dependency by definition,
// so a compile definition is always current. The git hash changes on every
// commit, and a configure-time stamp froze at whatever was checked out when
// the build directory was created — A-5, see cmake/stamp_git.cmake. Generating
// it on every build is the only form that is reliably true.
//
// The fallbacks keep editors and one-off compiles working without configuring.

#pragma once

#ifndef SAT_VERSION
#define SAT_VERSION "0.1.0"
#endif

// Short commit hash of the build, with a "-dirty" suffix when tracked files
// differed from HEAD. Written by cmake/stamp_git.cmake into the build tree.
//
// __has_include rather than a bare #include so that a compiler pointed at this
// header outside a configured build tree — an IDE indexing the repository, a
// one-off `g++ -Iinclude` — still works and simply reports "unknown".
#if defined(__has_include)
#  if __has_include(<sat/build_stamp.hpp>)
#    include <sat/build_stamp.hpp>
#  endif
#endif

#ifndef SAT_GIT_HASH
#define SAT_GIT_HASH "unknown"
#endif

// Human-readable product name used in CLI banners and log headers.
#ifndef SAT_PRODUCT_NAME
#define SAT_PRODUCT_NAME "SAT — Satellite Adaptive Tracker"
#endif
