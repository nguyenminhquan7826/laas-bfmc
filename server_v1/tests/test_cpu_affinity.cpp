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

    expect(parseCpuIndexConfig("1", cfg, reason), "CPU1 should parse");
    expect(cfg.enabled && cfg.cpu == 1, "CPU1 result mismatch");

    expect(parseCpuIndexConfig(" 3 ", cfg, reason), "trimmed CPU3 should parse");
    expect(cfg.enabled && cfg.cpu == 3, "CPU3 result mismatch");

    expect(parseCpuIndexConfig("off", cfg, reason), "off should parse");
    expect(!cfg.enabled && cfg.cpu == -1, "off result mismatch");

    expect(!parseCpuIndexConfig("1,2", cfg, reason), "CPU list must be rejected");
    expect(!parseCpuIndexConfig("-1", cfg, reason), "negative CPU must be rejected");
    expect(!parseCpuIndexConfig("abc", cfg, reason), "non-integer CPU must be rejected");
    expect(!parseCpuIndexConfig("", cfg, reason), "empty CPU must be rejected");

#ifdef __linux__
    cpu_set_t allowed{};
    CPU_ZERO(&allowed);
    expect(::sched_getaffinity(0, sizeof(allowed), &allowed) == 0,
           "sched_getaffinity should work on Linux runner");
    const std::string allowed_before = laas::formatCpuSet(allowed);

    expect(::unsetenv("LAAS_MAIN_CPU") == 0, "unsetenv should succeed");
    expect(laas::configureMainCpuAffinity(),
           "default configureMainCpuAffinity should succeed with affinity OFF");

    cpu_set_t after_default{};
    CPU_ZERO(&after_default);
    expect(::sched_getaffinity(0, sizeof(after_default), &after_default) == 0,
           "default affinity verification read should succeed");
    expect(laas::formatCpuSet(after_default) == allowed_before,
           "default affinity OFF must not change the Linux CPU mask");

    int selected = -1;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &allowed)) {
            selected = cpu;
            break;
        }
    }
    expect(selected >= 0, "Linux runner must expose at least one allowed CPU");

    const std::string selected_text = std::to_string(selected);
    expect(::setenv("LAAS_MAIN_CPU", selected_text.c_str(), 1) == 0,
           "setenv should succeed");
    expect(laas::configureMainCpuAffinity(),
           "explicit LAAS_MAIN_CPU should pin an allowed CPU");

    cpu_set_t actual{};
    CPU_ZERO(&actual);
    expect(::sched_getaffinity(0, sizeof(actual), &actual) == 0,
           "affinity verification read should succeed");
    expect(CPU_COUNT(&actual) == 1 && CPU_ISSET(selected, &actual),
           "runtime affinity must contain exactly the selected CPU");
#endif

    std::cout << "[PASS] main CPU affinity default OFF + explicit pin smoke\n";
    return 0;
}
