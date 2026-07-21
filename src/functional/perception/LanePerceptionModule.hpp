#pragma once

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"
#include "LaneDetectorCore.hpp"

namespace laas {

class LanePerceptionModule {
public:
    explicit LanePerceptionModule(const Config& config);

    bool process(const FrameMsg& input, LanePerceptionMsg& output);
    void reset();

private:
    Config config_;
    LaneDetectorCore detector_;
};

}  // namespace laas
