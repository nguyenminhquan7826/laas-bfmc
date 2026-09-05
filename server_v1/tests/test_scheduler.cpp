#include <cassert>
#include <cstdint>
#include <iostream>

#include "execution_control/Scheduler.hpp"

int main()
{
    using laas::PeriodicTimer;
    using laas::Scheduler;

    PeriodicTimer timer(20);
    timer.reset(100U);
    assert(!timer.ready(119U));
    assert(timer.ready(120U));

    timer.mark(127U);
    assert(!timer.ready(146U));
    assert(timer.ready(147U));

    timer.setPeriod(0);
    assert(timer.periodMs() == 1);
    timer.reset(200U);
    assert(!timer.ready(200U));
    assert(timer.ready(201U));

    Scheduler scheduler;
    scheduler.configure(33, 333, 33, 50, 50, 20, 100);
    assert(scheduler.camera.periodMs() == 33);
    assert(scheduler.yolo.periodMs() == 333);
    assert(scheduler.perception.periodMs() == 33);
    assert(scheduler.decision.periodMs() == 50);
    assert(scheduler.planning.periodMs() == 50);
    assert(scheduler.control.periodMs() == 20);
    assert(scheduler.logging.periodMs() == 100);

    scheduler.reset(1000U);
    assert(!scheduler.control.ready(1019U));
    assert(scheduler.control.ready(1020U));
    assert(!scheduler.logging.ready(1099U));
    assert(scheduler.logging.ready(1100U));

    std::cout << "[PASS] Scheduler periodic semantics\n";
    return 0;
}
