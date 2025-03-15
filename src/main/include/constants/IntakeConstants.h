#pragma once 

#include <units/voltage.h>
#include <units/time.h>

namespace constants
{
    namespace intake
    {
        constexpr units::volt_t intakePower = 6.0_V;
        constexpr units::volt_t outtakePower = -6.0_V;

        constexpr units::millisecond_t detectionDelay = 50_ms;
    }
}