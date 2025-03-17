#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

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
    rightMotorConfig.Inverted(false).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    rightMotorConfig.encoder.PositionConversionFactor(positionConversionFactor.value())
        .VelocityConversionFactor(velocityConversionFactor.value());

    m_rightMotor->Configure(rightMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

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

    m_pid->SetTolerance(constants::elevator::positionTolerance);

    m_limitSwitch = std::make_unique<frc::DigitalInput>(ports::dio::elevatorLimitSwitch);
}

void Elevator::Periodic() 
{
    // if(AtBottom())
    //     fmt::print("Elevator at bottom\n");

    // fmt::print("Test ");
    if(AtBottom() && m_leftMotor->Get() < 0 && m_rightMotor->Get() < 0)
    {
        fmt::print("Zeroing elevator\n");
        ZeroMotors();
        m_leftEncoder->SetPosition(0);
        m_rightEncoder->SetPosition(0);
    }
    else 
    {
        units::meter_t currentPosition = GetPosition();
        units::meter_t targetPosition = m_pid->GetSetpoint().position;
        units::meters_per_second_t targetVelocity = m_pid->GetSetpoint().velocity;

        units::volt_t outputFF = m_feedforward->Calculate(targetVelocity);

        units::volt_t outputPID { m_pid->Calculate(currentPosition) };
        units::volt_t output = 0.0_V;

        if(m_pid->AtGoal())
            output = outputFF;
        else 
            output = outputFF + outputPID;

        output = std::clamp(output, -constants::elevator::maxVoltage, constants::elevator::maxVoltage);
        
        fmt::print("Elevator position: {}, velocity: {}, target position: {}, target velocity: {}, output: {}\n",
                   GetPosition().value(),
                   GetVelocity().value(),
                   targetPosition.value(),
                   targetVelocity.value(),
                   output.value());

        // Set output voltage
        SetVoltage(output);
    }

    frc::SmartDashboard::PutNumber("Elevator Left Encoder", m_leftEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("Elevator Right Encoder", m_rightEncoder->GetPosition());
}

void Elevator::SetTargetHeight(units::meter_t height)
{
    m_pid->SetGoal(std::clamp(height, constants::elevator::positionMin, constants::elevator::positionMax));
}

bool Elevator::AtTargetHeight() const
{
    return m_pid->AtGoal();
}

units::meter_t Elevator::GetPosition() const
{
    units::meter_t leftPosition { m_leftEncoder->GetPosition() };
    units::meter_t rightPosition { m_rightEncoder->GetPosition() };
    units::meter_t averagePosition = (leftPosition + rightPosition) / 2.0;
    return averagePosition;
}

units::meters_per_second_t Elevator::GetVelocity() const
{
    units::meters_per_second_t leftVelocity { m_leftEncoder->GetVelocity() };
    units::meters_per_second_t rightVelocity { m_rightEncoder->GetVelocity() };
    units::meters_per_second_t averageVelocity = (leftVelocity + rightVelocity) / 2.0;
    return averageVelocity;
}

bool Elevator::AtBottom() const 
{
    return m_limitSwitch->Get();
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

std::unique_ptr<frc2::sysid::SysIdRoutine> Elevator::GetSysIdRoutine()
{
    auto routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config(0.5_V / 1.0_s, 1.0_V, 8.0_s, nullptr),
        frc2::sysid::Mechanism (
            [this] (units::volt_t voltage) { SetVoltage(voltage); },
            [this] (frc::sysid::SysIdRoutineLog* log) {
                units::volt_t batteryVoltage = frc::RobotController::GetBatteryVoltage();

                log->Motor("left-motor")
                    .voltage(m_leftMotor->Get() * batteryVoltage)
                    .position(units::meter_t { m_leftEncoder->GetPosition() })
                    .velocity(units::meters_per_second_t { m_leftEncoder->GetVelocity() });

                log->Motor("right-motor")
                    .voltage(m_rightMotor->Get() * batteryVoltage)
                    .position(units::meter_t { m_rightEncoder->GetPosition() })
                    .velocity(units::meters_per_second_t { m_rightEncoder->GetVelocity() });
            },
            this
        )
    );

    return routine;
}