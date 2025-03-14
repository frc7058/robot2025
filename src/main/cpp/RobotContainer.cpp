// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/JoystickButton.h>

#include "commands/DriveCommand.h"

RobotContainer::RobotContainer() {
    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kLeftBumper)
        .OnTrue(frc2::cmd::RunOnce([this] { m_elevator.SetVoltage(2.0_V); }, {}))
        .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {} ));

    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kRightBumper)
        .OnTrue(frc2::cmd::RunOnce([this] { m_elevator.SetVoltage(-2.0_V); }, {}))
        .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {} ));

    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kA)
      .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetVoltage(2.0_V); }, {}))
      .OnFalse(frc2::cmd::RunOnce([this] { m_arm.ZeroMotors(); }, {}));

    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kB)
      .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetVoltage(-2.0_V); }, {}))
      .OnFalse(frc2::cmd::RunOnce([this] { m_arm.ZeroMotors(); }, {}));

    // ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    ConfigureDriveControls();
}

void RobotContainer::ConfigureDriveControls() {
    m_driveBase.SetDefaultCommand(DriveCommand(&m_driveBase, m_driveController));

    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
        .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_driveBase.ZeroHeading(); m_driveBase.SetNavXHeading(0_rad); }, {}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
