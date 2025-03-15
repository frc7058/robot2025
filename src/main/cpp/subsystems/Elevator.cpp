#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Elevator.h"
#include "constants/ElevatorConstants.h"
#include "constants/Ports.h"
#include "lib/MotorConfig.h"

Elevator::Elevator() 
{
    units::meter_t positionConversionFactor = constants::elevator::sprocketCircumference / constants::elevator::gearRatio;
    units::meter_t velocityConversionFactor = positionConversionFactor / 60.0;

    m_leftMotor = std::make_unique<rev::spark::SparkMax>(ports::elevator::leftMotorCAN, rev::spark::SparkMax::MotorType::kBrushless);
    m_leftEncoder = std::make_unique<rev::spark::SparkRelativeEncoder>(m_leftMotor->GetEncoder());
    
    rev::spark::SparkMaxConfig leftMotorConfig;
    leftMotorConfig.Inverted(true).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    leftMotorConfig.encoder.PositionConversionFactor(positionConversionFactor.value())
        .VelocityConversionFactor(velocityConversionFactor.value());

    m_leftMotor->Configure(leftMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

    m_rightMotor = std::make_unique<rev::spark::SparkMax>(ports::elevator::rightMotorCAN, rev::spark::SparkMax::MotorType::kBrushless);
    m_rightEncoder = std::make_unique<rev::spark::SparkRelativeEncoder>(m_rightMotor->GetEncoder());

    rev::spark::SparkMaxConfig rightMotorConfig;
    rightMotorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    rightMotorConfig.encoder.PositionConversionFactor(positionConversionFactor.value())
        .VelocityConversionFactor(velocityConversionFactor.value());

    m_leftMotor->Configure(leftMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

    m_feedforward = std::make_unique<frc::ElevatorFeedforward>(
        constants::elevator::feedforward::staticFriction,
        constants::elevator::feedforward::gravity,
        constants::elevator::feedforward::velocity,
        constants::elevator::feedforward::acceleration);

    m_pid = std::make_unique<frc::ProfiledPIDController<units::meters>>(
        constants::elevator::pid::p,
        constants::elevator::pid::i,
        constants::elevator::pid::d,
        frc::TrapezoidProfile<units::meters>::Constraints(
            constants::elevator::pid::maxVelocity,
            constants::elevator::pid::maxAcceleration
        ));
}

void Elevator::Periodic() 
{
    frc::SmartDashboard::PutNumber("Elevator Left Encoder", m_leftEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("Elevator Right Encoder", m_leftEncoder->GetPosition());
}

void Elevator::SetVoltage(units::volt_t voltage) 
{
    m_leftMotor->SetVoltage(voltage);
    m_rightMotor->SetVoltage(voltage);
}

void Elevator::ZeroMotors()
{
    m_leftMotor->SetVoltage(0_V);
    m_rightMotor->SetVoltage(0_V);
}