#pragma once

#include <algorithm>
#include <cmath>

namespace laas {
namespace steering_calibration {

// Vehicle steering calibration for the shared Cybertruck 1:10 mechanical
// platform. The thesis measured servo-relative angle phi against left/right
// front-wheel angles at phi = 0,5,...,30 deg. The two wheel angles were first
// reduced to the equivalent bicycle steering angle using the Ackermann
// cotangent average. We then fit a physically symmetric odd cubic around the
// verified servo-neutral command.
//
//   delta_bicycle_deg(phi) = A * phi + B * phi^3
//
// with phi in degrees relative to servo center. The fit is constrained to pass
// through the measured maximum bicycle-equivalent point at phi=30 deg.
constexpr float kServoTravelDeg = 30.0f;
constexpr float kBicycleSteeringMaxDeg = 22.835397f;
constexpr float kPolyLinear = 0.9666368131f;
constexpr float kPolyCubic = -0.0002282854502f;

inline float servoOffsetToBicycleSteeringDeg(float servo_offset_deg)
{
    const float phi = std::max(
        -kServoTravelDeg,
        std::min(servo_offset_deg, kServoTravelDeg));

    return kPolyLinear * phi +
           kPolyCubic * phi * phi * phi;
}

// The runtime controller produces bicycle-model steering angle delta, while the
// STM32 protocol expects an absolute servo command. Invert the monotonic cubic
// numerically instead of introducing a second approximate polynomial.
inline float bicycleSteeringDegToServoOffset(float steering_deg)
{
    if (!std::isfinite(steering_deg)) {
        return 0.0f;
    }

    const float sign = steering_deg < 0.0f ? -1.0f : 1.0f;
    const float target = std::min(std::fabs(steering_deg),
                                  kBicycleSteeringMaxDeg);

    if (target <= 1e-6f) {
        return 0.0f;
    }

    float lo = 0.0f;
    float hi = kServoTravelDeg;

    // 24 iterations give far finer resolution than the integer-degree servo
    // command used by the current STM32 protocol.
    for (int i = 0; i < 24; ++i) {
        const float mid = 0.5f * (lo + hi);
        const float value = servoOffsetToBicycleSteeringDeg(mid);

        if (value < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return sign * 0.5f * (lo + hi);
}

inline int bicycleSteeringDegToServoCommand(float steering_deg,
                                            float servo_center_deg,
                                            int servo_min_deg,
                                            int servo_max_deg)
{
    const float offset = bicycleSteeringDegToServoOffset(steering_deg);
    const int command = static_cast<int>(
        std::lround(servo_center_deg + offset));

    return std::max(servo_min_deg,
                    std::min(command, servo_max_deg));
}

}  // namespace steering_calibration
}  // namespace laas
