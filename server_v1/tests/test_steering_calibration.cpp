#include <cassert>
#include <cmath>
#include <iostream>

#include "laas_core/SteeringCalibration.hpp"

namespace {

bool near(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

}  // namespace

int main()
{
    using namespace laas::steering_calibration;

    // Ackermann bicycle-equivalent values derived from the supplied thesis
    // left/right steering table (Eq. 2.6 cotangent average).
    const float phi_deg[] = {
        0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f};
    const float bicycle_deg[] = {
        0.0f,
        4.998005f,
        9.629213f,
        13.807675f,
        17.384847f,
        20.314653f,
        22.835397f};

    for (int i = 0; i < 7; ++i) {
        const float predicted =
            servoOffsetToBicycleSteeringDeg(phi_deg[i]);
        assert(near(predicted, bicycle_deg[i], 0.30f));

        // Odd symmetry around the physical neutral command.
        const float negative =
            servoOffsetToBicycleSteeringDeg(-phi_deg[i]);
        assert(near(negative, -predicted, 1e-5f));

        // The numerical inverse must recover the forward polynomial itself
        // essentially exactly.
        const float round_trip =
            bicycleSteeringDegToServoOffset(predicted);
        assert(near(round_trip, phi_deg[i], 1e-3f));
    }

    assert(near(servoOffsetToBicycleSteeringDeg(0.0f), 0.0f, 1e-6f));
    assert(near(servoOffsetToBicycleSteeringDeg(30.0f),
                kBicycleSteeringMaxDeg,
                1e-4f));

    // When the raw thesis table is inverted through the fitted cubic rather
    // than through its own exact forward value, the worst observed servo-angle
    // discrepancy is about 0.52 deg (at the 25-deg sample), still below the
    // current STM32 integer-degree command resolution.
    for (int i = 0; i < 7; ++i) {
        const float recovered =
            bicycleSteeringDegToServoOffset(bicycle_deg[i]);
        assert(near(recovered, phi_deg[i], 0.55f));
    }

    // Current vehicle absolute servo calibration.
    assert(bicycleSteeringDegToServoCommand(0.0f, 75.0f, 45, 105) == 75);
    assert(bicycleSteeringDegToServoCommand(kBicycleSteeringMaxDeg,
                                            75.0f, 45, 105) == 105);
    assert(bicycleSteeringDegToServoCommand(-kBicycleSteeringMaxDeg,
                                            75.0f, 45, 105) == 45);

    // Commands beyond the calibrated bicycle range must saturate at the
    // physically verified servo command limits.
    assert(bicycleSteeringDegToServoCommand(40.0f, 75.0f, 45, 105) == 105);
    assert(bicycleSteeringDegToServoCommand(-40.0f, 75.0f, 45, 105) == 45);

    std::cout << "Steering polynomial calibration tests PASS\n";
    return 0;
}
