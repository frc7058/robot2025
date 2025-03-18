#include "commands/ArmCommands.h"
#include "constants/ArmConstants.h"

namespace ArmCommands
{
    frc2::CommandPtr ResetArm(Arm* arm)
    {
        return frc2::FunctionalCommand(
            // Initialize
            [arm] { arm->Reset(); },

            // Periodic
            [arm] {},

            // OnEnd
            [arm] (bool interrupted) { arm->StopResetting(); },

            // IsFinished
            [arm] { return arm->AtReferencePosition(); },

            {arm}
        ).WithTimeout(5.0_s);
    }

    frc2::CommandPtr SetAngle(Arm* arm, units::radian_t angle)
    {
        return frc2::cmd::RunOnce([arm, angle] { arm->SetTargetAngle(angle); })
            .AndThen(frc2::cmd::WaitUntil([arm] { return arm->AtTargetAngle(); }))
            .WithTimeout(4.0_s);
    }
}