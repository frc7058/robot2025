#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <numbers>

#include "subsystems/Arm.h"
#include "constants/ArmConstants.h"
#include "constants/Ports.h"
#include "lib/MotorConfig.h"

using ResetMode = rev::spark::SparkBase::ResetMode;
using PersistMode = rev::spark::SparkBase::PersistMode;

Arm::Arm()
{
    m_armMotor = std::make_unique<rev::spark::SparkMax>(ports::arm::motorCAN, rev::spark::SparkMax::MotorType::kBrushless);
    m_armEncoder = std::make_unique<rev::spark::SparkRelativeEncoder>(m_armMotor->GetEncoder());

    units::radian_t positionConversionFactor = units::radian_t {2.0 * std::numbers::pi} / constants::arm::armGearRatio;
    units::radian_t velocityConversionFactor = positionConversionFactor / 60.0;

    rev::spark::SparkMaxConfig motorConfig;
    motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    motorConfig.encoder.PositionConversionFactor(positionConversionFactor.value())
        .VelocityConversionFactor(velocityConversionFactor.value());

    m_armMotor->Configure(motorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

    m_feedforward = std::make_unique<frc::ArmFeedforward>(
        constants::arm::feedforward::staticFriction,
        constants::arm::feedforward::gravity,
        constants::arm::feedforward::velocity,
        constants::arm::feedforward::acceleration);

    m_pid = std::make_unique<frc::ProfiledPIDController<units::radians>>(
        constants::arm::pid::p,
        constants::arm::pid::i,
        constants::arm::pid::d,
        frc::TrapezoidProfile<units::radians>::Constraints(
            constants::arm::pid::maxVelocity,
            constants::arm::pid::maxAcceleration
        ));
}

void Arm::Periodic() 
{
    units::radian_t currentAngle = GetAngle();

    units::radian_t targetAngle = m_pid->GetSetpoint().position; 
    units::radians_per_second_t targetVelocity = m_pid->GetSetpoint().velocity;

    units::volt_t outputFF = m_feedforward->Calculate(targetAngle, targetVelocity);
    units::volt_t outputPID { m_pid->Calculate(currentAngle) };

    units::volt_t output = outputFF + outputPID;

    // Clamp output voltage
    output = std::clamp(output, -constants::arm::maxVoltage, constants::arm::maxVoltage);

    // Set output voltage
    // SetVoltage(output);

    frc::SmartDashboard::PutNumber("Arm angle", (units::degree_t { m_armEncoder->GetPosition() }).value());
    frc::SmartDashboard::PutNumber("Arm commanded angle", (units::degree_t { m_pid->GetGoal().position }).value());
    frc::SmartDashboard::PutNumber("Arm output voltage", output.value());
}

void Arm::SetTargetAngle(units::radian_t angle)
{
    angle = frc::AngleModulus(angle);
    m_pid->SetGoal(std::clamp(angle, constants::arm::minAngle, constants::arm::maxAngle));
}

units::radian_t Arm::GetAngle() const
{
    units::radian_t angle { m_armEncoder->GetPosition() };
    return frc::AngleModulus(angle);
}

units::radians_per_second_t Arm::GetVelocity() const
{
    return units::radians_per_second_t { m_armEncoder->GetVelocity() };
}

void Arm::SetVoltage(units::volt_t voltage)
{
    m_armMotor->SetVoltage(voltage);
}

void Arm::Zero()
{
    m_armMotor->SetVoltage(0_V);
}

std::unique_ptr<frc2::sysid::SysIdRoutine> Arm::GetSysIdRoutine()
{
    auto routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config(0.1_V / 1.0_s, 0.5_V, 5.0_s, nullptr),
        frc2::sysid::Mechanism (
            [this] (units::volt_t voltage) { SetVoltage(voltage); },
            [this] (frc::sysid::SysIdRoutineLog* log) {
                units::volt_t batteryVoltage = frc::RobotController::GetBatteryVoltage();

                log->Motor("arm-motor")
                    .voltage(m_armMotor->Get() * batteryVoltage)
                    .position(units::turn_t { GetAngle() })
                    .velocity(units::turns_per_second_t { GetVelocity() });
            },
            this
        )
    );

    units::second_t timeout = 2.0_s;

    return std::move(routine);
}