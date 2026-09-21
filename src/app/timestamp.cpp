// app/timestamp.cpp

#include "app/timestamp.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>

namespace sat {

std::string utc_timestamp_now() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    // 64, not 32. An ISO-8601 instant is 20 bytes and always will be, but GCC
    // cannot prove that: std::tm's fields are plain ints with no declared
    // range, so it must assume tm_year could print eleven digits and warns
    // that the ':' separators might not fit. Widening the buffer is cheaper
    // than arguing — it is a stack array in a function called once per run.
    char buf[64];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02dT%02d:%02d:%02dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}

}  // namespace sat
