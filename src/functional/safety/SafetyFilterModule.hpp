#pragma once

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

class SafetyFilterModule {
public:
    explicit SafetyFilterModule(const Config& config);

    ControlCmdMsg filter(const ControlCmdMsg& raw_cmd,
                         const BehaviorRequest& behavior,
                         const LanePerceptionMsg& lane,
                         const TrajectoryMsg& trajectory) const;

private:
    Config config_;

    int steeringToServo(float steering_deg) const;
};

}  // namespace laas