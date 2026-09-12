// CP 0.1 smoke test: the version macros must expand to a non-empty string.
// Real unit tests (doctest) arrive with later checkpoints; this keeps CI green now.

#include "sat/version.hpp"

#include <cstdio>
#include <cstring>

int main() {
    if (std::strlen(SAT_VERSION) == 0) {
        std::fprintf(stderr, "SAT_VERSION is empty\n");
        return 1;
    }
    if (std::strlen(SAT_PRODUCT_NAME) == 0) {
        std::fprintf(stderr, "SAT_PRODUCT_NAME is empty\n");
        return 1;
    }

    // Print so failures in CI logs are easy to spot.
    std::printf("smoke ok: %s version %s\n", SAT_PRODUCT_NAME, SAT_VERSION);
    return 0;
}