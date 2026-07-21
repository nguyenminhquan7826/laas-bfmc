#pragma once

#include "ControlModule.hpp"
#include "MpcController.hpp"
#include "../../laas_core/Config.hpp"

namespace laas {

class MpcControlModule : public IControlModule {
public:
    explicit MpcControlModule(const Config& config);

    bool process(const TrajectoryMsg& trajectory,
                 const LanePerceptionMsg& lane,
                 const BehaviorRequest& behavior,
                 ControlCmdMsg& command) override;

    const MpcState& lastState() const { return last_state_; }

private:
    std::vector<cv::Point> selectPath(const TrajectoryMsg& trajectory,
                                      const LanePerceptionMsg& lane) const;
    void updateScaleFromLaneWidth(const LanePerceptionMsg& lane);

private:
    Config config_;
    MpcController controller_;
    MpcState last_state_;
};

}  // namespace laas
