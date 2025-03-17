#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <numbers>

#include "subsystems/Arm.h"
#include "constants/ArmConstants.h"
#include "constants/Ports.h"
#include "lib/MotorConfig.h"

Arm::Arm()
{
    m_armMotor = std::make_unique<rev::spark::SparkMax>(ports::arm::motorCAN, rev::spark::SparkMax::MotorType::kBrushless);
    m_armEncoder = std::make_unique<rev::spark::SparkRelativeEncoder>(m_armMotor->GetEncoder());

    units::radian_t positionConversionFactor = units::radian_t {2.0 * std::numbers::pi} / constants::arm::armGearRatio;
    units::radian_t velocityConversionFactor = positionConversionFactor / 60.0;

    rev::spark::SparkMaxConfig motorConfig;
    motorConfig.Inverted(true);
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

    m_pid->SetTolerance(constants::arm::angleTolerance);

    m_limitSwitch = std::make_unique<frc::DigitalInput>(ports::dio::armLimitSwitch);
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

    // if limit switch is hit and arm going backwards, stop
    if (AtReferencePosition() && m_armMotor->Get() < 0)
    {
        fmt::print("Zeroing arm\n");
        Zero();
        m_resetting = false;
        m_armEncoder->SetPosition(0);
    }
    else if(m_pid->AtGoal())
    {
        // fmt::print("At goal\n");
        Zero();
    }
    else if(!m_resetting) // when resetting, no PID control
    {
        // Set output voltage
        SetVoltage(output);
    }

    // fmt::print("Error: {}\n", (units::degree_t { m_pid->GetPositionError() }).value());
    // fmt::print("Arm angle: {}, Arm commanded angle: {}, Arm output voltage: {}, Arm velocity: {}, Arm commanded velocity: {}\n",
    //     (units::degree_t { GetAngle() }).value(),
    //     (units::degree_t { m_pid->GetGoal().position }).value(),
    //     output.value(),
    //     (units::degrees_per_second_t { m_armEncoder->GetVelocity() }).value(),
    //     (units::degrees_per_second_t { m_pid->GetGoal().velocity }).value());

    frc::SmartDashboard::PutNumber("Arm angle", (units::degree_t { GetAngle() }).value());
    frc::SmartDashboard::PutNumber("Arm commanded angle", (units::degree_t { m_pid->GetGoal().position }).value());
    frc::SmartDashboard::PutNumber("Arm output voltage", output.value());
}

void Arm::SetTargetAngle(units::radian_t angle)
{
    angle = frc::AngleModulus(angle);
    m_pid->SetGoal(std::clamp(angle, constants::arm::minAngle, constants::arm::maxAngle));
}

bool Arm::AtTargetAngle() const
{
    return m_pid->AtGoal();
}

void Arm::Reset()
{
    if(AtReferencePosition())
        return;

    m_resetting = true;
    m_armMotor->SetVoltage(constants::arm::resetVoltage);
}

bool Arm::AtReferencePosition() const 
{
    return m_limitSwitch.get();
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
        frc2::sysid::Config(0.25_V / 1.0_s, 0.75_V, 7.0_s, nullptr),
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

    return routine;
}