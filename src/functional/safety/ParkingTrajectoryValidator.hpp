#pragma once

#include <string>

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

struct ParkingTrajectoryValidationResult {
    bool accepted = false;
    std::string reason;
};

class ParkingTrajectoryValidator {
public:
    explicit ParkingTrajectoryValidator(const Config& config);

    ParkingTrajectoryValidationResult validate(
        const ParkingTrajectoryMsg& trajectory,
        const VehiclePoseMsg& current_pose) const;

private:
    static double wrapAngle(double angle_rad);

    const Config& config_;
};

}  // namespace laas
