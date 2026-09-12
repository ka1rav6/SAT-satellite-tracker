// Version string for the SAT binary.
//
// CMake injects SAT_VERSION from the project() VERSION field so the printed
// string always matches the build. The fallback keeps editors and one-off
// compiles working without configuring first.

#pragma once

#ifndef SAT_VERSION
#define SAT_VERSION "0.1.0"
#endif

// Human-readable product name used in CLI banners and log headers.
#ifndef SAT_PRODUCT_NAME
#define SAT_PRODUCT_NAME "SAT — Satellite Adaptive Tracker"
#endif