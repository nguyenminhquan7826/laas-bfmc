#include <cstdlib>
#include <iostream>
#include <string>

#include "../../src/laas_core/CpuAffinity.hpp"

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << "\n";
        std::exit(1);
    }
}

}  // namespace

int main()
{
    using laas::CpuIndexConfig;
    using laas::parseCpuIndexConfig;

    CpuIndexConfig cfg;
    std::string reason;

    expect(!cfg.enabled && cfg.cpu == -1,
           "default CpuIndexConfig must represent affinity OFF");

    expect(parseCpuIndexConfig("1", cfg, reason), "explicit CPU should parse");
    expect(cfg.enabled && cfg.cpu == 1, "explicit CPU result mismatch");

    expect(parseCpuIndexConfig("off", cfg, reason), "off should parse");
    expect(!cfg.enabled && cfg.cpu == -1, "off result mismatch");

    expect(!parseCpuIndexConfig("1,2", cfg, reason), "CPU list must be rejected");
    expect(!parseCpuIndexConfig("abc", cfg, reason), "invalid CPU must be rejected");

#ifdef __linux__
    cpu_set_t allowed{};
    CPU_ZERO(&allowed);
    expect(::sched_getaffinity(0, sizeof(allowed), &allowed) == 0,
           "sched_getaffinity should work on Linux runner");
    const std::string allowed_before = laas::formatCpuSet(allowed);

    expect(::unsetenv("LAAS_MAIN_CPU") == 0, "unsetenv should succeed");
    expect(laas::configureMainCpuAffinity(),
           "default affinity OFF should succeed");

    cpu_set_t after_default{};
    CPU_ZERO(&after_default);
    expect(::sched_getaffinity(0, sizeof(after_default), &after_default) == 0,
           "default affinity verification should succeed");
    expect(laas::formatCpuSet(after_default) == allowed_before,
           "default affinity OFF must not change the CPU mask");

    int selected = -1;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &allowed)) {
            selected = cpu;
            break;
        }
    }
    expect(selected >= 0, "runner must expose an allowed CPU");

    const std::string selected_text = std::to_string(selected);
    expect(::setenv("LAAS_MAIN_CPU", selected_text.c_str(), 1) == 0,
           "setenv should succeed");
    expect(laas::configureMainCpuAffinity(),
           "diagnostic explicit pin should still work");

    cpu_set_t actual{};
    CPU_ZERO(&actual);
    expect(::sched_getaffinity(0, sizeof(actual), &actual) == 0,
           "explicit affinity verification should succeed");
    expect(CPU_COUNT(&actual) == 1 && CPU_ISSET(selected, &actual),
           "explicit pin must select exactly one CPU");
#endif

    std::cout << "[PASS] main affinity default OFF + diagnostic pin\n";
    return 0;
}
