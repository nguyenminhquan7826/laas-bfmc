#include "Scheduler.hpp"

#include <algorithm>

namespace laas {

PeriodicTimer::PeriodicTimer(int period_ms)
{
    setPeriod(period_ms);
}

void PeriodicTimer::setPeriod(int period_ms)
{
    period_ms_ = std::max(1, period_ms);
}

void PeriodicTimer::reset(uint64_t now_ms)
{
    last_run_ms_ = now_ms;
}

bool PeriodicTimer::ready(uint64_t now_ms) const
{
    return now_ms >= last_run_ms_ &&
           (now_ms - last_run_ms_) >= static_cast<uint64_t>(period_ms_);
}

void PeriodicTimer::mark(uint64_t now_ms)
{
    last_run_ms_ = now_ms;
}

void Scheduler::configure(int camera_ms,
                          int yolo_ms,
                          int perception_ms,
                          int decision_ms,
                          int planning_ms,
                          int control_ms,
                          int logging_ms)
{
    camera.setPeriod(camera_ms);
    yolo.setPeriod(yolo_ms);
    perception.setPeriod(perception_ms);
    decision.setPeriod(decision_ms);
    planning.setPeriod(planning_ms);
    control.setPeriod(control_ms);
    logging.setPeriod(logging_ms);
}

void Scheduler::reset(uint64_t now_ms)
{
    camera.reset(now_ms);
    yolo.reset(now_ms);
    perception.reset(now_ms);
    decision.reset(now_ms);
    planning.reset(now_ms);
    control.reset(now_ms);
    logging.reset(now_ms);
}

}  // namespace laas
