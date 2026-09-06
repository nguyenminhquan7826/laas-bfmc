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

    std::cout << "[PASS] main CPU affinity parser\n";
    return 0;
}
