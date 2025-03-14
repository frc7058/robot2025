
#pragma once

#include <frc2/command/Commands.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include "constants/DriveConstants.h"
#include "subsystems/DriveBase.h"
//#include "lib/Vision.h"

class DriveCommand : public frc2::CommandHelper<frc2::Command, DriveCommand>
{
public:
    explicit DriveCommand(DriveBase* driveBase, frc::XboxController& driveController);

    void Execute();

private:
    DriveBase* m_driveBase;

    frc::XboxController& m_driveController;
};