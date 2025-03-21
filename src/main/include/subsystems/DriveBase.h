#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <studica/AHRS.h>
#include <array>
#include <frc/SerialPort.h>

#include "lib/SwerveModule.h"
//#include "lib/NavX.h"
//#include "lib/Vision.h"


class DriveBase : public frc2::SubsystemBase 
{
public:
    DriveBase();

    void ConfigurePathPlanner();

    void Periodic() override;

    void Drive(units::meters_per_second_t velocityX, 
               units::meters_per_second_t velocityY, 
               units::radians_per_second_t angularVelocity, 
               bool fieldRelative);

    void Drive(frc::ChassisSpeeds chassisSpeeds);

    void Stop();

    void SetTargetModuleStates(const wpi::array<frc::SwerveModuleState, 4>& moduleStates);

    //void VisionUpdate();

    void TrackHeading(units::radian_t heading);
    void TrackObject(units::radian_t heading);
    void DisableTracking();
    bool IsTrackingEnabled() const;

    std::optional<units::radian_t> GetTargetRotationOverride();

    void SetDriveControlMode(ControlMode controlMode);
    ControlMode GetDriveControlMode() const;

    void SetNavXHeading(units::radian_t heading);

    units::radian_t GetHeading();
    frc::Rotation2d GetRotation2d();
    void ZeroHeading();

    frc::Pose2d GetPose() const;
    void ResetPose(frc::Pose2d pose);

    frc::ChassisSpeeds GetChassisSpeeds() const;

    wpi::array<frc::SwerveModuleState, 4> GetSwerveModuleStates() const;
    wpi::array<frc::SwerveModulePosition, 4> GetSwerveModulePositions() const;

    bool IsNavXAvailable();

    frc2::CommandPtr GetSysIdRoutine();

    void InitializePreferences();
    void LoadPreferences();

private:
    // Swerve modules (ordered clockwise starting at front left module)
    std::array<std::unique_ptr<SwerveModule>, 4> m_swerveModules {};

    // Swerve kinematics helper class 
    std::unique_ptr<frc::SwerveDriveKinematics<4>> m_kinematics {};

    // Swerve odometry
    std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> m_poseEstimator {};
    frc::Field2d m_field;

    units::radian_t m_navXOffset {0};

    // NavX IMU 
    studica::AHRS m_navX {studica::AHRS::NavXComType::kUSB1};

    // Vision class for AprilTag pose estimation
    // Vision& m_vision;

    // Driving control mode
    ControlMode m_controlMode {ControlMode::ClosedLoop};

    // PID controller to lock/maintain heading
    std::unique_ptr<frc::PIDController> m_headingPID {};
    // units::radian_t m_angleToObject {};
    bool m_tracking = false;

    std::unique_ptr<frc2::sysid::SysIdRoutine> m_sysIdRoutine;
};