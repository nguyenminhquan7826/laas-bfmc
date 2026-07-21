#pragma once

#include "ControlModule.hpp"
#include "PurePursuitController.hpp"
#include "../../laas_core/Config.hpp"

namespace laas {

class PurePursuitControlModule : public IControlModule {
public:
    explicit PurePursuitControlModule(const Config& config);

    bool process(const TrajectoryMsg& trajectory,
                 const LanePerceptionMsg& lane,
                 const BehaviorRequest& behavior,
                 ControlCmdMsg& command) override;

private:
    std::vector<cv::Point> selectPath(const TrajectoryMsg& trajectory,
                                      const LanePerceptionMsg& lane) const;
    void updateScaleFromLaneWidth(const LanePerceptionMsg& lane);

private:
    Config config_;
    PurePursuitController controller_;
};

}  // namespace laas
