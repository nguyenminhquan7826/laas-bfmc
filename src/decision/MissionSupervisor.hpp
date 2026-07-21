#pragma once

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"
#include "../laas_core/Time.hpp"

namespace laas {

class MissionSupervisor {
public:
    explicit MissionSupervisor(const Config& config);

    BehaviorRequest update(const LanePerceptionMsg& lane,
                           const ObstacleMsg& obstacle,
                           bool user_run_request);

private:
    Config config_;
    bool emergency_stop_latched_ = false;
};

}  // namespace laas
