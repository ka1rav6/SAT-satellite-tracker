// app/resources.cpp

#include "app/resources.hpp"

#include <thread>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <psapi.h>
#elif defined(__unix__) || defined(__APPLE__)
#  include <sys/resource.h>
#  include <sys/time.h>
#endif

namespace sat {

unsigned hardware_threads() noexcept {
    return std::thread::hardware_concurrency();
}

ResourceUsage current_resource_usage() noexcept {
    ResourceUsage u;

#if defined(_WIN32)
    FILETIME created{}, exited{}, kernel{}, user{};
    if (GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user)) {
        // FILETIME counts 100-nanosecond intervals.
        auto to_seconds = [](const FILETIME& f) {
            const uint64_t ticks =
                (static_cast<uint64_t>(f.dwHighDateTime) << 32) | f.dwLowDateTime;
            return static_cast<double>(ticks) * 1e-7;
        };
        u.cpu_user_s   = to_seconds(user);
        u.cpu_system_s = to_seconds(kernel);
        u.available    = true;
    }
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        u.peak_rss_bytes = static_cast<uint64_t>(pmc.PeakWorkingSetSize);
        u.available      = true;
    }

#elif defined(__unix__) || defined(__APPLE__)
    rusage r{};
    if (getrusage(RUSAGE_SELF, &r) == 0) {
        u.cpu_user_s = static_cast<double>(r.ru_utime.tv_sec)
                     + static_cast<double>(r.ru_utime.tv_usec) * 1e-6;
        u.cpu_system_s = static_cast<double>(r.ru_stime.tv_sec)
                       + static_cast<double>(r.ru_stime.tv_usec) * 1e-6;
        // ru_maxrss IS NOT THE SAME UNIT ON EVERY PLATFORM, and getting this
        // wrong by 1024x in a performance log is the kind of error nobody
        // notices until someone sizes a machine from it. Linux reports
        // kilobytes; macOS and the BSDs report bytes. Documented in each
        // platform's getrusage(2) and different in both.
#  if defined(__APPLE__)
        u.peak_rss_bytes = static_cast<uint64_t>(r.ru_maxrss);
#  else
        u.peak_rss_bytes = static_cast<uint64_t>(r.ru_maxrss) * 1024ULL;
#  endif
        u.available = true;
    }
#endif

    return u;
}

}  // namespace sat
