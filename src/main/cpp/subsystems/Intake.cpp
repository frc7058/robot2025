#include "subsystems/Intake.h"
#include "constants/IntakeConstants.h"
#include "constants/Ports.h"
#include "lib/MotorConfig.h"

Intake::Intake()
{
    m_intakeMotor = std::make_unique<rev::spark::SparkMax>(ports::intake::intakeMotorCAN, rev::spark::SparkMax::MotorType::kBrushless);

    rev::spark::SparkMaxConfig intakeMotorConfig;
    intakeMotorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    m_intakeMotor->Configure(intakeMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

    m_photoElectricSensor = std::make_unique<frc::DigitalInput>(ports::dio::photoElectricSensor);
    m_debouncer = std::make_unique<frc::Debouncer>(constants::intake::detectionDelay, frc::Debouncer::DebounceType::kRising);
}

void Intake::SetVoltage(units::volt_t voltage)
{
    m_intakeMotor->SetVoltage(voltage);
}

void Intake::Zero()
{
    m_intakeMotor->SetVoltage(0_V);
}