#pragma once 

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/velocity.h>
#include <memory>
#include <string>
#include "MotorConfig.h"

enum class ControlMode
{
    OpenLoop,
    ClosedLoop
};

class SwerveModule
{
public:
    // Creates a swerve module
    SwerveModule(std::string name, int driveMotorCanID, int turnMotorCanID, int canCoderCanID, units::radian_t canCoderOffset);

    SwerveModule& operator=(const SwerveModule&) = delete;
    SwerveModule(const SwerveModule&) = delete;

    void Periodic();

    units::radian_t GetTurnAngle() const;
    units::radian_t GetTargetTurnAngle() const;
    units::meter_t GetDriveDistance() const;
    units::meters_per_second_t GetDriveVelocity() const;
    frc::SwerveModulePosition GetPosition() const;
    frc::SwerveModuleState GetState() const;

    void SetTurnAngle(units::radian_t angle);
    void SetDriveVelocity(units::meters_per_second_t velocity);
    void SetTargetState(frc::SwerveModuleState state);

    //void SetDriveIdleMode(rev::CANSparkBase::IdleMode mode);

    void SetTurnMotorInverted(bool inverted);
    void SetDriveMotorInverted(bool inverted);

    void SetControlMode(ControlMode controlMode);

    void UpdateTurnController(double p, double i, double d, double f, units::radians_per_second_t v, units::radians_per_second_squared_t a);
    void UpdateDriveController(double p, double i, double d, double ff_S, double ff_V);

    void UpdateTurnEncoderOffset(units::radian_t offset);
    
    std::unique_ptr<rev::spark::SparkMax> m_driveMotor {};
    std::unique_ptr<rev::spark::SparkMax> m_turnMotor {};

private:
    const std::string m_name {};

    rev::spark::SparkMaxConfig m_driveMotorConfig;
    rev::spark::SparkMaxConfig m_turnMotorConfig;

    std::unique_ptr<rev::spark::SparkRelativeEncoder> m_driveEncoder {};
    
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> m_turnEncoder {};
    units::radian_t m_encoderOffset {0};

    ControlMode m_controlMode {ControlMode::ClosedLoop};
    std::unique_ptr<frc::PIDController> m_drivePID {};
    std::unique_ptr<frc::ProfiledPIDController<units::radians>> m_turnPID {}; 
    double m_turnPID_F {};
    
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> m_driveFeedForward {};
};