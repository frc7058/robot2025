#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <numbers>

namespace constants 
{
    namespace arm
    {
        constexpr double armGearRatio = 84.0;

        constexpr units::radian_t minAngle = 0.0_deg;
        constexpr units::radian_t maxAngle = 135.0_deg;
        constexpr units::radian_t angleTolerance = 1.0_deg;

        constexpr units::volt_t maxVoltage = 4.0_V;
        constexpr units::volt_t resetVoltage = -2.0_V;

        constexpr units::radian_t outAngle = 60.0_deg;
        constexpr units::radian_t moveInAngle = 30.0_deg;
        constexpr units::radian_t inAngle = 0.0_deg;

        namespace feedforward 
        {
            constexpr units::volt_t staticFriction = 0.12011_V;
            constexpr units::volt_t gravity = 0.17917_V;
            constexpr auto velocity = 10.692_V * 1.0_s / 1.0_rad;
            constexpr auto acceleration = 2.4722_V * 1.0_s * 1.0_s / 1.0_rad;
        }

        namespace pid
        {
            constexpr double p = 32.0;
            constexpr double i = 0.0;
            constexpr double d = 0.0;

            constexpr units::radians_per_second_t maxVelocity {3 * std::numbers::pi};
            constexpr units::radians_per_second_squared_t maxAcceleration {5.0 * std::numbers::pi};
        }
    }
}