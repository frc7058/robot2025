#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/DigitalInput.h>
#include <rev/SparkMax.h>
#include <units/voltage.h>
#include <units/length.h>
#include <memory>

enum class ElevatorStage {
    Bottom,
    Stage1,
    Stage2,
    Stage3,
    Stage4
};

class Elevator : public frc2::SubsystemBase 
{
public:
    Elevator();

    void Periodic() override;

    void SetTargetStage(ElevatorStage stage);

    units::meter_t GetPosition() const;
    units::meters_per_second_t GetVelocity() const;

    std::unique_ptr<frc2::sysid::SysIdRoutine> GetSysIdRoutine();

    // Hide these eventually
    void SetVoltage(units::volt_t voltage);
    void ZeroMotors();

private:
    std::unique_ptr<rev::spark::SparkMax> m_leftMotor;
    std::unique_ptr<rev::spark::SparkMax> m_rightMotor;   

    std::unique_ptr<rev::spark::SparkRelativeEncoder> m_rightEncoder;
    std::unique_ptr<rev::spark::SparkRelativeEncoder> m_leftEncoder;

    std::unique_ptr<frc::ElevatorFeedforward> m_feedforward;
    std::unique_ptr<frc::ProfiledPIDController<units::meters>> m_pid;

    std::unique_ptr<frc::DigitalInput> m_limitSwitch;
};