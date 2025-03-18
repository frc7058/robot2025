#pragma once

#include <frc2/command/Commands.h>
#include <units/length.h>
#include "subsystems/Elevator.h"
#include "subsystems/Arm.h"

namespace ElevatorCommands
{
    // Sets the elevator to a specific height
    frc2::CommandPtr SetHeight(Elevator* elevator, Arm* arm, units::meter_t height);
}