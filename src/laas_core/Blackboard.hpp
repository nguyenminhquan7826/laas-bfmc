#pragma once

#include <mutex>
#include "Messages.hpp"

namespace laas {

class Blackboard {
public:
    void setFrame(const FrameMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_ = msg;
    }

    void setLane(const LanePerceptionMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        lane_ = msg;
    }

    void setObstacle(const ObstacleMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        obstacle_ = msg;
    }

    void setBehavior(const BehaviorRequest& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        behavior_ = msg;
    }

    void setTrajectory(const TrajectoryMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = msg;
    }

    void setRawCommand(const ControlCmdMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        raw_cmd_ = msg;
    }

    void setSafeCommand(const ControlCmdMsg& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        safe_cmd_ = msg;
    }

    FrameMsg frame() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frame_;
    }

    LanePerceptionMsg lane() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lane_;
    }

    ObstacleMsg obstacle() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return obstacle_;
    }

    BehaviorRequest behavior() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return behavior_;
    }

    TrajectoryMsg trajectory() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return trajectory_;
    }

    ControlCmdMsg rawCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return raw_cmd_;
    }

    ControlCmdMsg safeCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return safe_cmd_;
    }

private:
    mutable std::mutex mutex_;

    FrameMsg frame_;
    LanePerceptionMsg lane_;
    ObstacleMsg obstacle_;
    BehaviorRequest behavior_;
    TrajectoryMsg trajectory_;
    ControlCmdMsg raw_cmd_;
    ControlCmdMsg safe_cmd_;
};

}  // namespace laas
