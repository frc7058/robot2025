#pragma once

#include <frc2/command/Commands.h>
#include "subsystems/Arm.h"

namespace ArmCommands
{
    // Runs the arm backwards until the limit switch is hit
    frc2::CommandPtr ResetArm(Arm* arm);
}