#include "subsystems/DriveBase.h"

#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "constants/Ports.h"
#include "constants/DriveConstants.h"
#include "constants/AutoConstants.h"
#include "constants/GeneralConstants.h"
#include "lib/Util.h"
using namespace rev::spark;
using namespace pathplanner;

DriveBase::DriveBase()
{
    fmt::print("\nInitializing DriveBase...\n");

    m_swerveModules[0] = std::make_unique<SwerveModule>(
        "Swerve Module (FL)", 
        ports::drive::driveMotorCAN::frontLeft,
        ports::drive::turnMotorCAN::frontLeft,
        ports::drive::CANCoder::frontLeft,
        constants::drive::encoderOffsets::frontLeft);

    m_swerveModules[1] = std::make_unique<SwerveModule>(
        "Swerve Module (FR)", 
        ports::drive::driveMotorCAN::frontRight, 
        ports::drive::turnMotorCAN::frontRight, 
        ports::drive::CANCoder::frontRight,
        constants::drive::encoderOffsets::frontRight);
    m_swerveModules[1]->SetDriveMotorInverted(true);

    m_swerveModules[2] = std::make_unique<SwerveModule>(
        "Swerve Module (BL)", 
        ports::drive::driveMotorCAN::backLeft, 
        ports::drive::turnMotorCAN::backLeft, 
        ports::drive::CANCoder::backLeft,
        constants::drive::encoderOffsets::backLeft);

    m_swerveModules[3] = std::make_unique<SwerveModule>(
        "Swerve Module (BR)", 
        ports::drive::driveMotorCAN::backRight,
        ports::drive::turnMotorCAN::backRight,
        ports::drive::CANCoder::backRight,
        constants::drive::encoderOffsets::backRight);
    m_swerveModules[3]->SetDriveMotorInverted(true);
     
    m_kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
        frc::Translation2d( constants::drive::moduleDistanceX,  constants::drive::moduleDistanceY),
        frc::Translation2d( constants::drive::moduleDistanceX, -constants::drive::moduleDistanceY),
        frc::Translation2d(-constants::drive::moduleDistanceX,  constants::drive::moduleDistanceY),
        frc::Translation2d(-constants::drive::moduleDistanceX, -constants::drive::moduleDistanceY));

    m_poseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
        *m_kinematics, 
        GetHeading(), 
        GetSwerveModulePositions(), 
        frc::Pose2d(
            frc::Translation2d(0.0_m, 0.0_m),
            frc::Rotation2d(0.0_rad)
        ));

    units::radian_t headingTolerance = constants::drive::headingPID::tolerance;
    m_headingPID = std::make_unique<frc::PIDController>(
        constants::drive::headingPID::p,
        constants::drive::headingPID::i,
        constants::drive::headingPID::d
    );
    m_headingPID->SetTolerance(headingTolerance.value());
    m_headingPID->EnableContinuousInput(-constants::pi, constants::pi);

    InitializePreferences();

    pathplanner::AutoBuilder::configure(
        // Get pose
        [this] () { return this->GetPose(); },

        // Reset pose
        [this] (frc::Pose2d pose) { 
            this->ResetPose(pose); 
        },

        // Get robot-relative speeds
        [this] () { return this->GetChassisSpeeds(); },

        // Drive
        [this] (frc::ChassisSpeeds robotRelativeSpeeds) { this->Drive(robotRelativeSpeeds); },

        // Path follower config
        constants::autonomous::pathFollowerConfig,

        constants::autonomous::config,

        // Boolean supplier for path mirroring
        [] () {
            auto alliance = frc::DriverStation::GetAlliance();

            if(alliance)
                return alliance.value() == frc::DriverStation::Alliance::kRed;

            return false;
        },

        this
    );

    // pathplanner::PPHolonomicDriveController::setRotationTargetOverride([this] () {
    //    return GetTargetRotationOverride();
    // });
    
    frc::SmartDashboard::PutData("Field", &m_field);
    
    fmt::print("DriveBase Initialization complete\n\n");
}

std::optional<units::radian_t> DriveBase::GetTargetRotationOverride()
{
    // std::optional<units::radian_t> angleToTarget = m_vision.GetTargetAngle();

    // if(angleToTarget)
    // {
    //     return frc::AngleModulus(angleToTarget.value() + GetHeading());
    // }

    return std::nullopt;
}

void DriveBase::Periodic()
{
    LoadPreferences();

    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->Periodic();
    }

    m_poseEstimator->Update(GetRotation2d(), GetSwerveModulePositions());

   // VisionUpdate();

    frc::SmartDashboard::PutNumber("X", GetPose().X().value());
    frc::SmartDashboard::PutNumber("Y", GetPose().Y().value());
    frc::SmartDashboard::PutBoolean("NavX calibrating", m_navX.IsCalibrating());
    frc::SmartDashboard::PutBoolean("NavX connected", m_navX.IsConnected());

    m_field.SetRobotPose(GetPose());
}

void DriveBase::Drive(units::meters_per_second_t velocityX, units::meters_per_second_t velocityY, units::radians_per_second_t angularVelocity, bool fieldRelative)
{
    //fmt::print("Vx: {} m/s, Vy: {} m/s, Vangular: {} rad/s\n", velocityX.value(), velocityY.value(), angularVelocity.value());

    frc::ChassisSpeeds chassisSpeeds { velocityX, velocityY, angularVelocity };

    if(fieldRelative)
    {
        frc::Rotation2d robotRotation = GetRotation2d();
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(velocityX, velocityY, angularVelocity, robotRotation);
    }

    Drive(chassisSpeeds);
}

void DriveBase::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    // If tracking an object, rotate towards
    if(m_tracking)
    {
        units::radian_t heading = GetHeading();
        units::radians_per_second_t outputAngularVelocity { -m_headingPID->Calculate(heading.value()) };
        outputAngularVelocity += constants::drive::headingPID::ff * util::sign(outputAngularVelocity);

        fmt::print("Output angular velocity: {}\n", outputAngularVelocity);

        if(m_headingPID->AtSetpoint())
        {
            fmt::print("At setpoint\n");
            chassisSpeeds.omega = 0.0_rad_per_s;
        }
        else 
        {
            chassisSpeeds.omega = std::min(outputAngularVelocity, constants::drive::maxAngularVelocity);
            // fmt::print("Tracking output: {}\n", chassisSpeeds.omega);
            // units::degree_t degreeError = units::radian_t {m_headingPID->GetPositionError()};         
            // fmt::print("Error: {} ({}), output: {}", m_headingPID->GetPositionError(), degreeError, outputAngularVelocity);
        }
    } 

    units::radians_per_second_t angularVelocity = chassisSpeeds.omega;
    chassisSpeeds.omega *= constants::drive::angularVelocityFudgeFactor;
    chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.02_s);
    chassisSpeeds.omega = angularVelocity;

    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = m_kinematics->ToSwerveModuleStates(chassisSpeeds);
    m_kinematics->DesaturateWheelSpeeds(&swerveModuleStates, constants::drive::maxDriveVelocity);

    for(size_t index = 0; index < m_swerveModules.size(); index++)
    {
        frc::SwerveModuleState& moduleState = swerveModuleStates[index];
        units::radian_t currentTurnAngle = m_swerveModules[index]->GetTurnAngle();

        // Optimize module states to minimize the required turn angle
        moduleState.Optimize(frc::Rotation2d(currentTurnAngle));
    
        // Scale module speed by the cosine of angle error (to some power)
        // Reduces motor movement while modules are still turning to their target angle
        if(constants::drive::enableCosineScaling)
        {
            double errorCosine = units::math::cos(currentTurnAngle - moduleState.angle.Radians());
            double scalingFactor = std::pow(std::abs(errorCosine), constants::drive::cosineScalingExponent);

            moduleState.speed *= scalingFactor;
        }
    }

    SetTargetModuleStates(swerveModuleStates);
}

void DriveBase::Stop()
{
    Drive(frc::ChassisSpeeds {0_mps, 0_mps, 0_rad_per_s});
}

void DriveBase::SetTargetModuleStates(const wpi::array<frc::SwerveModuleState, 4>& moduleStates)
{
    for(size_t moduleIndex = 0; moduleIndex < 4; moduleIndex++)
    {
        frc::SwerveModuleState state = moduleStates[moduleIndex];
        m_swerveModules[moduleIndex]->SetTargetState(state);
    }
}

//{
   // frc::Pose3d currentPose(GetPose());
   // std::vector<std::optional<VisionPoseResult>> estimatedPoses = m_vision.GetEstimatedPoses(currentPose);

   // for(std::optional<VisionPoseResult>& visionResult : estimatedPoses)
   // {
    //   if(visionResult.has_value())
     //   {
      //      frc::Pose2d estimatedPose2d = visionResult->estimatedPose.estimatedPose.ToPose2d();
       //     units::second_t timestamp = visionResult->estimatedPose.timestamp;
        //    
        //    m_poseEstimator->AddVisionMeasurement(estimatedPose2d, timestamp, visionResult->standardDeviations);
       // }
   // }
//}

void DriveBase::TrackHeading(units::radian_t heading)
{
    heading = frc::AngleModulus(heading);
    m_headingPID->SetSetpoint(heading.value());
    m_tracking = true;
}

// void DriveBase::TrackObject(units::radian_t heading)
// {
//     heading = frc::AngleModulus(heading);
//     m_angleToObject = heading;
//     //m_headingPID->SetSetpoint(heading.value());
//     m_tracking = true;
// }

void DriveBase::DisableTracking()
{
    m_tracking = false;
}

bool DriveBase::IsTrackingEnabled() const
{
    return m_tracking;
}

void DriveBase::SetDriveControlMode(ControlMode controlMode)
{
    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->SetControlMode(controlMode);
    }

    m_controlMode = controlMode;
}

ControlMode DriveBase::GetDriveControlMode() const
{
    return m_controlMode;
}

void DriveBase::SetNavXHeading(units::radian_t heading)
{
    m_navXOffset = heading;
    m_navX.ZeroYaw();
}

units::radian_t DriveBase::GetHeading()
{
    units::degree_t heading {m_navX.GetYaw()};

    return frc::AngleModulus(units::radian_t {heading} + m_navXOffset);
}

frc::Rotation2d DriveBase::GetRotation2d()
{
    return m_navX.GetRotation2d().RotateBy(frc::Rotation2d(m_navXOffset));
}

void DriveBase::ZeroHeading()
{
    m_navX.ZeroYaw();
}

frc::Pose2d DriveBase::GetPose() const
{
    return m_poseEstimator->GetEstimatedPosition();
}

void DriveBase::ResetPose(frc::Pose2d pose)
{
    SetNavXHeading(pose.Rotation().Radians());
    m_poseEstimator->ResetPosition(GetHeading(), GetSwerveModulePositions(), pose);
}

frc::ChassisSpeeds DriveBase::GetChassisSpeeds() const
{
    return m_kinematics->ToChassisSpeeds(GetSwerveModuleStates());
}

wpi::array<frc::SwerveModuleState, 4> DriveBase::GetSwerveModuleStates() const
{
    return {
        m_swerveModules[0]->GetState(),
        m_swerveModules[1]->GetState(),
        m_swerveModules[2]->GetState(),
        m_swerveModules[3]->GetState()
    };
}

wpi::array<frc::SwerveModulePosition, 4> DriveBase::GetSwerveModulePositions() const 
{
    return {
        m_swerveModules[0]->GetPosition(),
        m_swerveModules[1]->GetPosition(),
        m_swerveModules[2]->GetPosition(),
        m_swerveModules[3]->GetPosition()
    };
}

bool DriveBase::IsNavXAvailable()
{
    return m_navX.IsConnected() && !m_navX.IsCalibrating();
}

frc2::CommandPtr DriveBase::GetSysIdRoutine()
{
    m_sysIdRoutine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config(std::nullopt, std::nullopt, std::nullopt, nullptr),

        frc2::sysid::Mechanism(
            [this] (units::volt_t voltage)
            {
                for(const auto& swerveModule : m_swerveModules)
                {
                    swerveModule->SetTurnAngle(0_rad);
                    swerveModule->m_driveMotor->SetVoltage(voltage); 
                }
            },

            [this] (frc::sysid::SysIdRoutineLog* log)
            {
                units::volt_t batteryVoltage = frc::RobotController::GetBatteryVoltage();

                log->Motor("Drive Motor")
                    .voltage(m_swerveModules[0]->m_driveMotor->Get() * batteryVoltage)
                    .position(m_swerveModules[0]->GetDriveDistance())
                    .velocity(m_swerveModules[0]->GetDriveVelocity());
            },

            this
        )
    );

    units::second_t timeout = 5.0_s;

    return this->RunOnce([this] { 
            for(const auto& swerveModule : m_swerveModules)
            {
                swerveModule->SetTurnAngle(0_rad);
            }
        })
        .AndThen(frc2::cmd::Wait(1.0_s))
        .AndThen(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kForward))
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Quasistatic(frc2::sysid::Direction::kReverse))
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kForward))
            .WithTimeout(timeout)
        .AndThen(frc2::cmd::Wait(5.0_s))
        .AndThen(m_sysIdRoutine->Dynamic(frc2::sysid::Direction::kReverse))
            .WithTimeout(timeout);
}

void DriveBase::InitializePreferences()
{
    frc::Preferences::InitDouble(constants::drive::preferences::driveP_Key, constants::drive::drivePID::p);
    frc::Preferences::InitDouble(constants::drive::preferences::driveI_Key, constants::drive::drivePID::i);
    frc::Preferences::InitDouble(constants::drive::preferences::driveD_Key, constants::drive::drivePID::d);
    frc::Preferences::InitDouble(constants::drive::preferences::driveFF_S_Key, constants::drive::driveFF::s.value());
    frc::Preferences::InitDouble(constants::drive::preferences::driveFF_V_Key, constants::drive::driveFF::v.value());
    
    frc::Preferences::InitDouble(constants::drive::preferences::turnP_Key, constants::drive::turnPID::p);
    frc::Preferences::InitDouble(constants::drive::preferences::turnI_Key, constants::drive::turnPID::i);
    frc::Preferences::InitDouble(constants::drive::preferences::turnD_Key, constants::drive::turnPID::d);
    frc::Preferences::InitDouble(constants::drive::preferences::turnF_Key, constants::drive::turnPID::f);
    frc::Preferences::InitDouble(constants::drive::preferences::turnV_Key, constants::drive::turnPID::maxVelocity.value());
    frc::Preferences::InitDouble(constants::drive::preferences::turnA_Key, constants::drive::turnPID::maxAcceleration.value());

    LoadPreferences();
}

void DriveBase::LoadPreferences()
{
    double driveP = frc::Preferences::GetDouble(constants::drive::preferences::driveP_Key);
    double driveI = frc::Preferences::GetDouble(constants::drive::preferences::driveI_Key);
    double driveD = frc::Preferences::GetDouble(constants::drive::preferences::driveD_Key);
    double driveFF_S = frc::Preferences::GetDouble(constants::drive::preferences::driveFF_S_Key);
    double driveFF_V = frc::Preferences::GetDouble(constants::drive::preferences::driveFF_V_Key);

    double turnP = frc::Preferences::GetDouble(constants::drive::preferences::turnP_Key);
    double turnI = frc::Preferences::GetDouble(constants::drive::preferences::turnI_Key);
    double turnD = frc::Preferences::GetDouble(constants::drive::preferences::turnD_Key);
    double turnF = frc::Preferences::GetDouble(constants::drive::preferences::turnF_Key);
    units::radians_per_second_t turnV { frc::Preferences::GetDouble(constants::drive::preferences::turnV_Key) };
    units::radians_per_second_squared_t turnA { frc::Preferences::GetDouble(constants::drive::preferences::turnA_Key) };

    for(std::unique_ptr<SwerveModule>& swerveModule : m_swerveModules)
    {
        swerveModule->UpdateDriveController(driveP, driveI, driveD, driveFF_S, driveFF_V);
        swerveModule->UpdateTurnController(turnP, turnI, turnD, turnF, turnV, turnA);
    }
}