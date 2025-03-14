#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <numbers>

namespace constants {
    namespace elevator {
        constexpr units::meter_t sprocketDiameter = 0.0508_m; // 2 inches
        constexpr units::meter_t sprocketCircumference = sprocketDiameter * std::numbers::pi;
        constexpr double gearRatio = 12.0;

        namespace feedforward {
            constexpr units::volt_t staticFriction = 0.0_V;
            constexpr units::volt_t gravity = 0.0_V;
            constexpr auto velocity = 0.0_V * 1.0_s / 1.0_m;
            constexpr auto acceleration = 0.0_V * 1.0_s * 1.0_s / 1.0_m;
        }

        namespace pid {
            constexpr double p = 0.0;
            constexpr double i = 0.0;
            constexpr double d = 0.0;

            constexpr units::meters_per_second_t maxVelocity = 3.0_mps;
            constexpr units::meters_per_second_squared_t maxAcceleration = 12.0_mps_sq;
        }
    }
}