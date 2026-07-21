#pragma once

#include "../../laas_core/Messages.hpp"

namespace laas {

class IControlModule {
public:
    virtual ~IControlModule() = default;
    virtual bool process(const TrajectoryMsg& trajectory,
                         const LanePerceptionMsg& lane,
                         const BehaviorRequest& behavior,
                         ControlCmdMsg& command) = 0;
};

}  // namespace laas
