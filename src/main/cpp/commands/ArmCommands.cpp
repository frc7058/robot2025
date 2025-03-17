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
            [arm] (bool interrupted) { arm->Zero(); },

            // IsFinished
            [arm] { return arm->AtReferencePosition(); },

            {arm}
        ).WithTimeout(5.0_s);
    }
}