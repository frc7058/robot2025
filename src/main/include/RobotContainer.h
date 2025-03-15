// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/XboxController.h>

#include "subsystems/DriveBase.h"
#include "subsystems/Elevator.h"
#include "subsystems/Arm.h"

class RobotContainer 
{
  public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

  private:
    void ConfigureBindings();
    void ConfigureDriveControls();

    frc::XboxController m_driveController {0};

    DriveBase m_driveBase {};
    Elevator m_elevator {};
    Arm m_arm {};

    std::unique_ptr<frc2::sysid::SysIdRoutine> m_sysIdRoutine;
};
