#pragma once

#include <frc2/command/Commands.h>
#include <units/angle.h>
#include "subsystems/Arm.h"

namespace ArmCommands
{
    // Runs the arm backwards until the limit switch is hit
    frc2::CommandPtr ResetArm(Arm* arm);

    // Sets the arm to a specific angle
    frc2::CommandPtr SetAngle(Arm* arm, units::radian_t angle);
}