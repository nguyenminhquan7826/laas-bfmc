#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#ifdef __linux__
#include <sched.h>
#include <unistd.h>
#endif

namespace laas {

struct CpuIndexConfig {
    bool enabled{false};
    int cpu{-1};
};

inline std::string trimCpuConfig(std::string value)
{
    const auto not_space = [](unsigned char ch) {
        return !std::isspace(ch);
    };

    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
                value.end());
    return value;
}

inline bool parseCpuIndexConfig(const std::string& text,
                                CpuIndexConfig& out,
                                std::string& reason)
{
    const std::string value = trimCpuConfig(text);
    if (value.empty()) {
        reason = "empty_cpu_config";
        return false;
    }

    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });

    if (lower == "off" || lower == "none" || lower == "disabled") {
        out.enabled = false;
        out.cpu = -1;
        reason = "ok";
        return true;
    }

    std::size_t used = 0U;
    int cpu = -1;
    try {
        cpu = std::stoi(value, &used, 10);
    } catch (...) {
        reason = "cpu_not_integer";
        return false;
    }

    if (used != value.size()) {
        reason = "cpu_has_trailing_characters";
        return false;
    }
    if (cpu < 0) {
        reason = "cpu_must_be_nonnegative";
        return false;
    }
#ifdef CPU_SETSIZE
    if (cpu >= CPU_SETSIZE) {
        reason = "cpu_exceeds_CPU_SETSIZE";
        return false;
    }
#endif

    out.enabled = true;
    out.cpu = cpu;
    reason = "ok";
    return true;
}

#ifdef __linux__
inline std::string formatCpuSet(const cpu_set_t& set)
{
    std::ostringstream oss;
    bool first = true;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &set)) {
            continue;
        }
        if (!first) {
            oss << ',';
        }
        oss << cpu;
        first = false;
    }
    return first ? std::string("none") : oss.str();
}
#endif

// Measured Pi 5 default: leave laas_pp unrestricted. Whole-process pinning
// reduced control/vision performance after the control-thread split.
// LAAS_MAIN_CPU=<index> remains only for explicit diagnostics/benchmarks.
inline bool configureMainCpuAffinity()
{
    const char* env = std::getenv("LAAS_MAIN_CPU");
    const bool from_env = env && *env;
    const std::string requested = from_env ? env : "off";

    CpuIndexConfig config;
    std::string reason;
    if (!parseCpuIndexConfig(requested, config, reason)) {
        std::cerr << "[CPU] invalid LAAS_MAIN_CPU='" << requested
                  << "' reason=" << reason << "\n";
        return false;
    }

    if (!config.enabled) {
        std::cout << "[CPU] laas_pp affinity=OFF"
                  << " source=" << (from_env ? "LAAS_MAIN_CPU" : "default")
                  << "\n";
        return true;
    }

#ifndef __linux__
    std::cerr << "[CPU] CPU affinity requires Linux; requested CPU"
              << config.cpu << "\n";
    return false;
#else
    cpu_set_t allowed{};
    CPU_ZERO(&allowed);
    if (::sched_getaffinity(0, sizeof(allowed), &allowed) != 0) {
        std::cerr << "[CPU] sched_getaffinity failed\n";
        return false;
    }

    if (!CPU_ISSET(config.cpu, &allowed)) {
        std::cerr << "[CPU] requested CPU" << config.cpu
                  << " is not in current allowed mask="
                  << formatCpuSet(allowed) << "\n";
        return false;
    }

    cpu_set_t target{};
    CPU_ZERO(&target);
    CPU_SET(config.cpu, &target);

    if (::sched_setaffinity(0, sizeof(target), &target) != 0) {
        std::cerr << "[CPU] sched_setaffinity failed for CPU"
                  << config.cpu << "\n";
        return false;
    }

    cpu_set_t actual{};
    CPU_ZERO(&actual);
    if (::sched_getaffinity(0, sizeof(actual), &actual) != 0) {
        std::cerr << "[CPU] affinity verification failed\n";
        return false;
    }

    if (!CPU_ISSET(config.cpu, &actual) || CPU_COUNT(&actual) != 1) {
        std::cerr << "[CPU] affinity verification mismatch actual="
                  << formatCpuSet(actual) << "\n";
        return false;
    }

    std::cout << "[CPU] laas_pp affinity=" << config.cpu
              << " allowedBefore=" << formatCpuSet(allowed)
              << " source=" << (from_env ? "LAAS_MAIN_CPU" : "default")
              << "\n";
    return true;
#endif
}

}  // namespace laas
