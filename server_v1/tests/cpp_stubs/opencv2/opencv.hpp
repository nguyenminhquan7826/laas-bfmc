#pragma once

// Test-only stub used by the parking client CI harness.
// Production builds must use real OpenCV. ParkingServerClient/ParkingProtocol
// only need Messages.hpp to be parsable; they do not use image operations.

namespace cv {

struct Mat {};

struct Point {
    int x = 0;
    int y = 0;

    Point() = default;
    Point(int x_value, int y_value) : x(x_value), y(y_value) {}
};

struct Vec3f {
    float values[3];

    Vec3f(float a = 0.0F, float b = 0.0F, float c = 0.0F)
        : values{a, b, c}
    {
    }

    float& operator[](int index) { return values[index]; }
    const float& operator[](int index) const { return values[index]; }
};

}  // namespace cv
