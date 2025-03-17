#include "commands/ElevatorCommands.h"
#include "subsystems/Elevator.h"

namespace ElevatorCommands
{
    frc2::CommandPtr SetHeight(Elevator* elevator, units::meter_t height)
    {
        return frc2::FunctionalCommand(
            // Initialize
            [elevator, height] { elevator->SetTargetHeight(height); },

            // Periodic
            [elevator] {},

            // OnEnd
            [elevator] (bool interrupted) {},

            // IsFinished
            [elevator] { return elevator->AtTargetHeight(); },

            {elevator}   
        ).WithTimeout(4.0_s);
    }
}