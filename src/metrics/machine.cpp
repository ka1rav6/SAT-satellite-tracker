// metrics/machine.cpp — host probing for the provenance record (audit P3-5).
//
// See machine.hpp for why this exists. Every function here is allowed to fail
// and must not: a missing field becomes "unknown".

#include "metrics/machine.hpp"

#include "degrade/sensor_simd.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#if defined(_WIN32)
#  include <intrin.h>
#elif defined(__APPLE__)
#  include <sys/sysctl.h>
#endif

namespace sat {
namespace {

constexpr const char* kUnknown = "unknown";

std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

// ---------------------------------------------------------------------------
// CPU model name.
//
// Three platforms, three mechanisms, and none of them is allowed to throw.
// ---------------------------------------------------------------------------
std::string probe_cpu() {
#if defined(_WIN32)
    // CPUID leaves 0x80000002..4 hold the 48-byte brand string. This is the
    // same data the registry's ProcessorNameString carries, without needing
    // registry access.
    int regs[4] = {0, 0, 0, 0};
    __cpuid(regs, 0x80000000);
    if (static_cast<unsigned>(regs[0]) < 0x80000004u) return kUnknown;
    char brand[49] = {};
    for (unsigned leaf = 0; leaf < 3; ++leaf) {
        __cpuid(regs, static_cast<int>(0x80000002u + leaf));
        std::memcpy(brand + leaf * 16, regs, 16);
    }
    const std::string s = trim(brand);
    return s.empty() ? kUnknown : s;
#elif defined(__APPLE__)
    char buf[256] = {};
    size_t len = sizeof buf;
    if (sysctlbyname("machdep.cpu.brand_string", buf, &len, nullptr, 0) != 0) {
        return kUnknown;
    }
    const std::string s = trim(buf);
    return s.empty() ? kUnknown : s;
#else
    // Linux. "model name" is x86; "Model" is what aarch64 /proc/cpuinfo uses,
    // and CI's ARM runners would otherwise report unknown.
    std::ifstream in("/proc/cpuinfo");
    if (!in) return kUnknown;
    std::string line;
    std::string fallback;
    while (std::getline(in, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = trim(line.substr(0, colon));
        const std::string val = trim(line.substr(colon + 1));
        if (val.empty()) continue;
        if (key == "model name") return val;                 // x86, preferred
        if (key == "Model" || key == "Hardware" || key == "CPU implementer") {
            if (fallback.empty()) fallback = val;             // aarch64
        }
    }
    return fallback.empty() ? kUnknown : fallback;
#endif
}

constexpr const char* kOsName =
#if defined(_WIN32)
    "Windows";
#elif defined(__APPLE__)
    "macOS";
#elif defined(__linux__)
    "Linux";
#else
    kUnknown;
#endif

std::string probe_compiler() {
    std::ostringstream o;
    // Clang must be tested before GCC: clang defines __GNUC__ too, and a
    // build labelled "gcc 4.2.1" on a clang host is worse than no label.
#if defined(__clang__)
    o << "clang " << __clang_major__ << '.' << __clang_minor__ << '.'
      << __clang_patchlevel__;
#elif defined(_MSC_VER)
    o << "MSVC " << (_MSC_VER / 100) << '.' << (_MSC_VER % 100);
#elif defined(__GNUC__)
    o << "gcc " << __GNUC__ << '.' << __GNUC_MINOR__ << '.'
      << __GNUC_PATCHLEVEL__;
#else
    o << kUnknown;
#endif
    return o.str();
}

}  // namespace

std::string MachineSpec::one_line() const {
    std::ostringstream o;
    o << cpu << ", " << hardware_threads << " threads, "
      << (avx2_damage_chain ? "AVX2" : "scalar") << ", " << os << ", "
      << compiler << ", " << build_type;
    return o.str();
}

MachineSpec probe_machine() {
    MachineSpec m;
    m.cpu        = probe_cpu();
    m.os         = kOsName;
    m.compiler   = probe_compiler();
    m.build_type = SAT_BUILD_TYPE;

    m.hardware_threads = std::thread::hardware_concurrency();

    // The path ACTUALLY TAKEN. damage_chain_simd_available() already folds in
    // both the CPU's capability and --no-simd, which is the distinction that
    // matters: a run forced onto the scalar chain is several times slower and
    // without this field that reads as a tracker regression.
    m.avx2_damage_chain = damage_chain_simd_available();
    return m;
}

}  // namespace sat
