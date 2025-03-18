// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <cameraserver/CameraServer.h>

#include "commands/DriveCommand.h"
#include "commands/ArmCommands.h"
#include "commands/ElevatorCommands.h"
#include "constants/ElevatorConstants.h"
#include "constants/IntakeConstants.h"

RobotContainer::RobotContainer() {
    frc::CameraServer::StartAutomaticCapture().SetResolution(1280, 720);

    // ArmCommands::ResetArm(&m_arm).Unwrap()->Schedule();

   

    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kA)
    //   .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetVoltage(2.0_V); }, {}))
    //   .OnFalse(frc2::cmd::RunOnce([this] { m_arm.ZeroMotors(); }, {}));

    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kB)
    //   .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetVoltage(-2.0_V); }, {}))
    //   .OnFalse(frc2::cmd::RunOnce([this] { m_arm.ZeroMotors(); }, {}));

    frc2::JoystickButton(&m_intakeController, frc::XboxController::Button::kA)
      .OnTrue(ArmCommands::ResetArm(&m_arm));
    frc2::JoystickButton(&m_intakeController, frc::XboxController::Button::kX)
      .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetTargetAngle(60.0_deg); }, {}));
    frc2::JoystickButton(&m_intakeController, frc::XboxController::Button::kY)
      .OnTrue(frc2::cmd::RunOnce([this] { m_arm.SetTargetAngle(90.0_deg); }, {}));

    frc2::POVButton(&m_intakeController, 180)
      .OnTrue(ElevatorCommands::SetHeight(&m_elevator, &m_arm, constants::elevator::stages::bottomPosition));
    frc2::POVButton(&m_intakeController, 90)
       .OnTrue(ElevatorCommands::SetHeight(&m_elevator, &m_arm, constants::elevator::stages::stageOnePosition));
     frc2::POVButton(&m_intakeController, 270)
      .OnTrue(ElevatorCommands::SetHeight(&m_elevator, &m_arm, constants::elevator::stages::stageTwoPosition));
    frc2::POVButton(&m_intakeController, 0)
      .OnTrue(ElevatorCommands::SetHeight(&m_elevator, &m_arm, constants::elevator::stages::stageThreePosition));


    ////////////////////////////////////////////////////////////////////////////////////////////////

    //intake thing

    frc2::JoystickButton(&m_intakeController, frc::XboxController::Button::kRightBumper)
       .OnTrue(frc2::cmd::RunOnce([this] { m_intake.SetVoltage(constants::intake::intakePower); }, {}))
      .OnFalse(frc2::cmd::RunOnce([this] { m_intake.Zero(); }, {}));

    frc2::JoystickButton(&m_intakeController, frc::XboxController::Button::kLeftBumper)
      .OnTrue(frc2::cmd::RunOnce([this] { m_intake.SetVoltage(constants::intake::outtakePower); }, {}))
      .OnFalse(frc2::cmd::RunOnce([this] { m_intake.Zero(); }, {}));



//////////////////////////////////////////
    // m_sysIdRoutine = m_elevator.GetSysIdRoutine();

    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kA)
    //     .WhileTrue(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kForward))
    //     .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {})); 
    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kB)
    //     .WhileTrue(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kReverse))
    //     .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {})); 
    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
    //     .WhileTrue(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kForward))
    //     .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {})); 
    // frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kY)
    //     .WhileTrue(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kReverse))
    //     .OnFalse(frc2::cmd::RunOnce([this] { m_elevator.ZeroMotors(); }, {})); 

    ConfigureBindings();

    m_sysIdRoutine = m_arm.GetSysIdRoutine();

    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kA)
        .WhileTrue(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kForward))
        .OnFalse(frc2::cmd::RunOnce([this] { m_arm.Zero(); }, {})); 
    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kB)
        .WhileTrue(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kReverse))
        .OnFalse(frc2::cmd::RunOnce([this] { m_arm.Zero(); }, {})); 
    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kX)
        .WhileTrue(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kForward))
        .OnFalse(frc2::cmd::RunOnce([this] { m_arm.Zero(); }, {})); 
    frc2::JoystickButton(&m_driveController, frc::XboxController::Button::kY)
        .WhileTrue(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kReverse))
        .OnFalse(frc2::cmd::RunOnce([this] { m_arm.Zero(); }, {})); 
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
