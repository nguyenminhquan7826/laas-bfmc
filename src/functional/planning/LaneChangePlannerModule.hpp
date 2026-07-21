#pragma once

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"
#include "LaneChangePlanner.hpp"

namespace laas {

class LaneChangePlannerModule {
public:
    explicit LaneChangePlannerModule(const Config& config);

    bool process(const LanePerceptionMsg& lane,
                 const ObstacleMsg& obstacle,
                 const BehaviorRequest& behavior,
                 TrajectoryMsg& trajectory);

    const LaneChangePlanner& planner() const { return planner_; }

private:
    Config config_;
    LaneChangePlanner planner_;

    float obstacleDistanceForPlanner(const ObstacleMsg& obstacle,
                                     const BehaviorRequest& behavior) const;
};

}  // namespace laas
