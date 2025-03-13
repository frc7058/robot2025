#include "lib/SwerveModule.h"
#include "lib/Util.h"
#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

using ResetMode = rev::spark::SparkBase::ResetMode;
using PersistMode = rev::spark::SparkBase::PersistMode;

SwerveModule::SwerveModule(std::string name, int driveMotorCanID, int turnMotorCanID, int canCoderCanID, units::radian_t canCoderOffset)
    : m_name(name), m_encoderOffset(canCoderOffset)
{
    fmt::print("Initializing Swerve Module {}\n", name);

    // Drive motor
    m_driveMotor = std::make_unique<rev::spark::SparkMax>(driveMotorCanID, rev::spark::SparkMax::MotorType::kBrushless);

    units::meter_t positionConversionFactor = constants::drive::wheelCircumference / (constants::drive::driveGearRatio) * constants::drive::driveMeasurementFudgeFactor;
    units::meter_t velocityConversionFactor = positionConversionFactor / 60.0;

    m_driveMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    m_driveMotorConfig.encoder.CountsPerRevolution(constants::encoderCountsPerRev)
        .UvwAverageDepth(constants::drive::driveEncoderDepth)
        .UvwMeasurementPeriod(constants::drive::driveEncoderPeriod)
        .PositionConversionFactor(positionConversionFactor.value())
        .VelocityConversionFactor(velocityConversionFactor.value());

    m_driveMotor->Configure(m_driveMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);

    // Turn motor 
    m_turnMotor = std::make_unique<rev::spark::SparkMax>(turnMotorCanID, rev::spark::SparkMax::MotorType::kBrushless);
    m_turnEncoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(canCoderCanID);

    m_turnMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake).Inverted(true);

m_turnMotor->Configure(m_turnMotorConfig, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters);
    m_drivePID = std::make_unique<frc::PIDController>(
        constants::drive::drivePID::p,
        constants::drive::drivePID::i,
        constants::drive::drivePID::d);
    m_drivePID->SetSetpoint(0);

    m_turnPID = std::make_unique<frc::ProfiledPIDController<units::radians>>(
        constants::drive::turnPID::p,
        constants::drive::turnPID::i,
        constants::drive::turnPID::d,
        frc::TrapezoidProfile<units::radians>::Constraints {
           constants::drive::turnPID::maxVelocity,
           constants::drive::turnPID::maxAcceleration
        });
    m_turnPID->SetGoal(0_rad);
    m_turnPID->EnableContinuousInput(-constants::piRadians, constants::piRadians);
    m_turnPID->SetTolerance(constants::drive::turnPID::tolerance);
    m_turnPID_F = constants::drive::turnPID::f;

    m_driveFeedForward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        constants::drive::driveFF::s,
        constants::drive::driveFF::v,
        constants::drive::driveFF::a);

    // Check if the motors or encoders have detected faults
    bool faults_detected = ((m_driveMotor->GetStickyFaults() , m_turnMotor->GetStickyFaults() , m_turnEncoder->GetStickyFaultField().GetValue()) == 0); //// , used to be |
    bool okay = m_driveMotor && m_turnMotor && m_turnEncoder && m_driveEncoder && m_drivePID && m_turnPID && m_driveFeedForward;

    if(okay)
    {
        if(!faults_detected)
        {
            fmt::print(" [OK]\n");
        }
        else 
        {
            fmt::print(" [FAULTS DETECTED]\n");
        }
    }
    else
    {
        fmt::print(" [ERROR]\n");
    }
}

void SwerveModule::Periodic()
{
    units::meters_per_second_t velocity = GetDriveVelocity();
    units::meters_per_second_t targetVelocity { m_drivePID->GetSetpoint() };

    units::volt_t driveOutputFF { m_driveFeedForward->Calculate(targetVelocity) };
    units::volt_t driveOutputPID { m_drivePID->Calculate(velocity.value()) };

    units::volt_t driveOutput = 0_V;

    if(m_controlMode == ControlMode::OpenLoop)
    {
        driveOutput = driveOutputFF;
    }
    else if(m_controlMode == ControlMode::ClosedLoop)
    {
        driveOutput = driveOutputFF + driveOutputPID;
    }

    driveOutput = std::clamp(driveOutput, -constants::drive::maxDriveVoltage, constants::drive::maxDriveVoltage);
    m_driveMotor->SetVoltage(driveOutput);
   
    // if(m_name == "Swerve Module (FL)")
    //     fmt::print("Target velocity: {} m/s, velocity: {} m/s, voltage: {}V\n", targetVelocity.value(), velocity.value(), driveOutput.value());

    frc::SmartDashboard::PutNumber(m_name + " Speed (m/s)", velocity.value());
    frc::SmartDashboard::PutNumber(m_name + " Commanded Speed (m/s)", targetVelocity.value());
    frc::SmartDashboard::PutNumber(m_name + " Output Voltage", driveOutput.value());

    units::radian_t angle = GetTurnAngle();
    units::volt_t turnOutput { m_turnPID->Calculate(angle) };

    if(m_turnPID->AtGoal())
    {
        m_turnMotor->StopMotor();
    }
    else 
    {
        turnOutput += units::volt_t { m_turnPID_F * util::sign(turnOutput.value()) };
        turnOutput = std::clamp(turnOutput, -constants::drive::maxTurnVoltage, constants::drive::maxTurnVoltage);

        m_turnMotor->SetVoltage(turnOutput);
    }
}   

units::radian_t SwerveModule::GetTurnAngle() const
{
    units::radian_t angle { m_turnEncoder->GetAbsolutePosition().GetValue() };
    return frc::AngleModulus(angle + m_encoderOffset);
}

units::radian_t SwerveModule::GetTargetTurnAngle() const
{
    units::radian_t angle = m_turnPID->GetGoal().position;
    return frc::AngleModulus(angle);
}

units::meter_t SwerveModule::GetDriveDistance() const
{
    return units::meter_t { m_driveEncoder->GetPosition() };
}

units::meters_per_second_t SwerveModule::GetDriveVelocity() const
{
    return units::meters_per_second_t { m_driveEncoder->GetVelocity() };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    return frc::SwerveModulePosition { GetDriveDistance(), GetTurnAngle() };
}

frc::SwerveModuleState SwerveModule::GetState() const
{
    return frc::SwerveModuleState { GetDriveVelocity(), GetTurnAngle() };
}

void SwerveModule::SetTurnMotorInverted(bool inverted)
{
    m_turnMotorConfig.Inverted(inverted);
    m_turnMotor->Configure(m_turnMotorConfig, ResetMode::kNoResetSafeParameters, PersistMode::kNoPersistParameters);
}

void SwerveModule::SetDriveMotorInverted(bool inverted)
{
    m_driveMotorConfig.Inverted(inverted);
    m_driveMotor->Configure(m_driveMotorConfig, ResetMode::kNoResetSafeParameters, PersistMode::kNoPersistParameters);
}

void SwerveModule::SetControlMode(ControlMode controlMode)
{
    m_controlMode = controlMode;
}

void SwerveModule::SetTurnAngle(units::radian_t angle)
{
    m_turnPID->SetGoal(frc::AngleModulus(angle));
}

void SwerveModule::SetDriveVelocity(units::meters_per_second_t velocity)
{
    m_drivePID->SetSetpoint(velocity.value());
}

void SwerveModule::SetTargetState(frc::SwerveModuleState state)
{
    SetTurnAngle(state.angle.Radians());
    SetDriveVelocity(state.speed);
}

void SwerveModule::UpdateTurnController(double p, double i, double d, double f, units::radians_per_second_t v, units::radians_per_second_squared_t a)
{
    m_turnPID->SetPID(p, i, d);
    m_turnPID->SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints { v, a });
    m_turnPID_F = f;
}

void SwerveModule::UpdateDriveController(double p, double i, double d, double ff_S, double ff_V)
{
    m_drivePID->SetPID(p, i, d);
    m_driveFeedForward.reset(new frc::SimpleMotorFeedforward<units::meters>(
        ff_S * 1.0_V,
        ff_V * 1.0_V * 1.0_s / 1.0_m,
        constants::drive::driveFF::a
    ));
}

void SwerveModule::UpdateTurnEncoderOffset(units::radian_t offset)
{
    m_encoderOffset = offset;
}