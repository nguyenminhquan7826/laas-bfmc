#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "../laas_core/Config.hpp"

namespace laas {

struct LocalParkingPlannerProcessResult {
    std::uint64_t decision_id{0};
    bool timed_out{false};
    int exit_code{-1};
    std::string output;
    std::string reason;
};

// Runs the Python Hybrid A* worker outside the cooperative Executive loop.
// It has no UART access and communicates through one request/one response
// NDJSON exchange. Linux fork/exec happens in the background worker thread;
// the 20 ms control thread never waits for planning.
class LocalParkingPlannerWorker {
public:
    explicit LocalParkingPlannerWorker(const Config& config);
    ~LocalParkingPlannerWorker();

    bool start(std::uint64_t decision_id,
               const std::string& request_line,
               std::string& reason);

    bool poll(LocalParkingPlannerProcessResult& result);
    bool busy() const { return running_.load(); }
    void stop();

private:
    LocalParkingPlannerProcessResult runProcess(
        std::uint64_t decision_id,
        const std::string& request_line);

    Config config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
    mutable std::mutex result_mutex_;
    bool result_ready_{false};
    LocalParkingPlannerProcessResult result_;
};

}  // namespace laas
