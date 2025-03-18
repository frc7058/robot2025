#include "commands/ElevatorCommands.h"
#include "commands/ArmCommands.h"
#include "constants/ElevatorConstants.h"
#include "constants/ArmConstants.h"

namespace ElevatorCommands
{
    static units::radian_t GetArmAngle(units::meter_t height)
    {
        // Elevator is below the "in" position: move outward
        if (height < constants::elevator::stages::armInPosition)
            return constants::arm::outAngle;
        else // Elevator is above the "in" position: move inward
            return constants::arm::moveInAngle;
    }

    frc2::CommandPtr SetHeight(Elevator* elevator, Arm* arm, units::meter_t height)
    {
        units::radian_t startAngle = GetArmAngle(elevator->GetPosition());
        fmt::print("Height: {}, start angle: {}\n", elevator->GetPosition().value(), (units::degree_t {startAngle}).value());

        return ArmCommands::SetAngle(arm, startAngle)
            .AndThen(frc2::FunctionalCommand(
                // Initialize
                [elevator, height] { elevator->SetTargetHeight(height); },

                // Periodic
                [elevator, arm] {
                    units::meter_t height = elevator->GetPosition();
                    units::radian_t targetAngle = GetArmAngle(height);
                    fmt::print("Height: {}, target angle: {}, angle: {}\n", elevator->GetPosition().value(), (units::degree_t{targetAngle}).value(), (units::degree_t {arm->GetAngle()}).value());

                    arm->SetTargetAngle(targetAngle);
                },

                // OnEnd
                [elevator] (bool interrupted) {},

                // IsFinished
                [elevator] { return elevator->AtTargetHeight(); },

                {elevator, arm}   
            ).WithTimeout(4.0_s));
    }
}