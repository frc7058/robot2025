#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Arm.h"
// #include "constants/ArmConstants.h"
#include "constants/Ports.h"

using ResetMode = rev::spark::SparkBase::ResetMode;
using PersistMode = rev::spark::SparkBase::PersistMode;

Arm::Arm()
{
    m_motor = std::make_unique<rev::spark::SparkMax>(ports::arm::motorCAN, rev::spark::SparkMax::MotorType::kBrushless);
    m_encoder = std::make_unique<rev::spark::SparkRelativeEncoder>(m_motor->GetEncoder());

    rev::spark::SparkMaxConfig motorConfig;
    motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    m_motor->Configure(motorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);
}

void Arm::Periodic() 
{
    frc::SmartDashboard::PutNumber("Arm Encoder", m_encoder->GetPosition());
}

void Arm::SetVoltage(units::volt_t voltage) 
{
    m_motor->SetVoltage(voltage);
}

void Arm::ZeroMotors()
{
    m_motor->SetVoltage(0_V);
}