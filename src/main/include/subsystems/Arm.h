#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/DigitalInput.h>
#include <rev/SparkMax.h>
#include <units/voltage.h>
#include <units/length.h>
#include <memory>

class Arm : public frc2::SubsystemBase 
{
public:
    Arm();

    void Periodic() override;

    // Hide these eventually
    void SetVoltage(units::volt_t voltage);
    void ZeroMotors();

private:
    std::unique_ptr<rev::spark::SparkMax> m_motor;
    std::unique_ptr<rev::spark::SparkRelativeEncoder> m_encoder;

    std::unique_ptr<frc::ArmFeedforward> m_feedforward;
    std::unique_ptr<frc::ProfiledPIDController<units::meters>> m_pid;

    std::unique_ptr<frc::DigitalInput> m_limitSwitch;
};