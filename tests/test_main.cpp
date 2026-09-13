// tests/test_main.cpp — the single doctest entry point, linked into every suite.
//
// Defining DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN in exactly one translation unit
// keeps the framework's implementation out of every other test file, which is
// most of why doctest compiles as fast as it does.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
