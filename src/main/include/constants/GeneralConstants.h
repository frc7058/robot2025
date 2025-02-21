#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <frc/util/Color8Bit.h>

namespace constants 
{
    constexpr uint64_t navXTimeoutSeconds = 30.0;

    constexpr double pi = 3.14159265358979323846;
    constexpr units::radian_t piRadians { pi };

    constexpr bool enableSysId = false;

    // Neo and Neo 550 encoder counts per revolution
    constexpr int encoderCountsPerRev = 42;

    namespace controls 
    {
        constexpr double joystickDeadband = 0.08;
        constexpr double axisDeadband = 0.05;
    }
}