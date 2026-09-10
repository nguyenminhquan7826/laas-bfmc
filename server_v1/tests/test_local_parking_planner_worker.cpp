#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "logical_robot/LocalParkingPlannerWorker.hpp"

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << "\n";
        std::exit(1);
    }
}

laas::LocalParkingPlannerProcessResult waitFor(
    laas::LocalParkingPlannerWorker& worker)
{
    laas::LocalParkingPlannerProcessResult result;
    for (int i = 0; i < 300; ++i) {
        if (worker.poll(result)) {
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(false, "worker result timed out in test harness");
    return result;
}

void writeScript(const std::string& path, const std::string& body)
{
    std::ofstream stream(path);
    require(stream.good(), "temporary worker script must open");
    stream << body;
    stream.close();
}

}  // namespace

int main()
{
    const std::string script = "/tmp/laas_local_planner_worker_test.py";
    laas::Config config;
    config.parking.enable_local_parking_planner = true;
    config.parking.local_planner_python = "python3";
    config.parking.local_planner_script = script;
    config.parking.local_planner_root = "/tmp";
    config.parking.local_planner_timeout_ms = 2000;
    config.parking.local_planner_max_output_bytes = 4096;

    writeScript(
        script,
        "import sys\n"
        "sys.stdin.readline()\n"
        "print('{\\\"type\\\":\\\"ok\\\"}', flush=True)\n");

    laas::LocalParkingPlannerWorker worker(config);
    std::string reason;
    require(worker.start(7U, "{\"request\":true}", reason),
            "worker must start");
    const laas::LocalParkingPlannerProcessResult success = waitFor(worker);
    require(success.decision_id == 7U, "decision id must survive worker");
    require(success.exit_code == 0, "worker must exit successfully");
    require(success.reason == "ok", "successful worker reason must be ok");
    require(success.output.find("\"type\":\"ok\"") != std::string::npos,
            "worker must capture stdout");

    config.parking.local_planner_timeout_ms = 50;
    writeScript(
        script,
        "import sys, time\n"
        "sys.stdin.readline()\n"
        "time.sleep(1)\n"
        "print('{}', flush=True)\n");
    laas::LocalParkingPlannerWorker timeout_worker(config);
    require(timeout_worker.start(8U, "{}", reason),
            "timeout worker must start");
    const laas::LocalParkingPlannerProcessResult timeout =
        waitFor(timeout_worker);
    require(timeout.timed_out, "worker must enforce planning timeout");
    require(timeout.reason == "local_planner_timeout",
            "timeout reason must fail closed");

    std::remove(script.c_str());
    std::cout << "[PASS] asynchronous local parking planner worker\n";
    return 0;
}
