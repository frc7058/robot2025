#pragma once

#include <frc2/command/Commands.h>
#include <units/length.h>
#include "subsystems/Elevator.h"
#include "subsystems/Arm.h"

namespace ElevatorCommands
{
    // Sets the arm to a specific height
    frc2::CommandPtr SetHeight(Elevator* arm, Arm* arm, units::meter_t height);
}