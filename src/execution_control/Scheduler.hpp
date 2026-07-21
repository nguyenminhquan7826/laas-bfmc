#pragma once

#include <cstdint>

namespace laas {

class PeriodicTimer {
public:
    PeriodicTimer() = default;
    explicit PeriodicTimer(int period_ms);

    void setPeriod(int period_ms);
    void reset(uint64_t now_ms);
    bool ready(uint64_t now_ms) const;
    void mark(uint64_t now_ms);

    int periodMs() const { return period_ms_; }

private:
    int period_ms_ = 20;
    uint64_t last_run_ms_ = 0;
};

struct Scheduler {
    PeriodicTimer camera;
    PeriodicTimer yolo;
    PeriodicTimer perception;
    PeriodicTimer decision;
    PeriodicTimer planning;
    PeriodicTimer control;
    PeriodicTimer logging;

    void configure(int camera_ms,
                   int yolo_ms,
                   int perception_ms,
                   int decision_ms,
                   int planning_ms,
                   int control_ms,
                   int logging_ms);

    void reset(uint64_t now_ms);
};

}  // namespace laas
