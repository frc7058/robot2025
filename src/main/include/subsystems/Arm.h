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

    void SetTargetAngle(units::radian_t angle);

    units::radian_t GetAngle() const;
    units::radians_per_second_t GetVelocity() const;

    std::unique_ptr<frc2::sysid::SysIdRoutine> GetSysIdRoutine();

private:
    void SetVoltage(units::volt_t voltage);
    void Zero();

private:
    std::unique_ptr<rev::spark::SparkMax> m_armMotor;
    std::unique_ptr<rev::spark::SparkRelativeEncoder> m_armEncoder;

    std::unique_ptr<frc::ArmFeedforward> m_feedforward;
    std::unique_ptr<frc::ProfiledPIDController<units::radians>> m_pid;

    std::unique_ptr<frc::DigitalInput> m_limitSwitch;
};