#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/filter/Debouncer.h>
#include <rev/SparkMax.h>
#include <units/voltage.h>
#include <memory>

class Intake : public frc2::SubsystemBase 
{
public:
    Intake();

    // void Periodic() override;

    void SetVoltage(units::volt_t voltage);
    void Zero();

private:
    std::unique_ptr<rev::spark::SparkMax> m_intakeMotor;

    std::unique_ptr<frc::DigitalInput> m_photoElectricSensor;
    std::unique_ptr<frc::Debouncer> m_debouncer;
};