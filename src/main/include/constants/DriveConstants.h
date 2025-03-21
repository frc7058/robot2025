#pragma once 

#include <numbers>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace constants 
{
    namespace drive 
    {
        // Motor voltage limits
        constexpr units::volt_t maxDriveVoltage = 10.0_V; // og 12v
        constexpr units::volt_t maxTurnVoltage = 8.0_V; // og 10v
        
        // Maximum drive velocities
        constexpr units::meters_per_second_t maxDriveVelocity = 4.5_mps; // og 4.5
        constexpr units::radians_per_second_t maxAngularVelocity {2.25 * std::numbers::pi}; // used to be 2.25

        constexpr units::meters_per_second_t slowMaxDriveVelocity = 1.5_mps;
        constexpr units::radians_per_second_t slowMaxAngularVelocity {1.0 * std::numbers::pi};

        // Swerve wheels inforxrmation
        constexpr units::meter_t wheelDiameter = 0.1016_m;
        constexpr units::meter_t wheelCircumference = wheelDiameter * std::numbers::pi;

        // Swerve module locations from robot center
        constexpr units::meter_t moduleDistanceX = 0.29845_m; 
        constexpr units::meter_t moduleDistanceY = 0.29845_m; 
        // constexpr units::meter_t radiusToModules = 0.38615_m; 
        
        constexpr double rampRateSeconds = 0.1;

        // Swerve drive gear ratio 
        constexpr double driveGearRatio = 6.75;

        constexpr double driveMeasurementFudgeFactor = 0.97; // To match expected real-world measurements
        constexpr double angularVelocityFudgeFactor = 6.0; // To account for lateral drift

        // Drive encoder measurement values
        constexpr uint32_t driveEncoderDepth = 8;
        constexpr uint32_t driveEncoderPeriod = 32;

        // Cosine scaling of drive motor speed
        constexpr bool enableCosineScaling = true;
        constexpr double cosineScalingExponent = 1.0;

        namespace encoderOffsets 
        {
            constexpr units::radian_t frontLeft = 1.24712638055_rad;
            constexpr units::radian_t frontRight = -0.62586416145_rad;
            constexpr units::radian_t backLeft = -1.88372840752_rad;
            constexpr units::radian_t backRight = 2.8854178620_rad;
        }

        // Turn motor profiled PIDF values
        namespace turnPID  
        {
            constexpr double p = 7.5;
            constexpr double i = 0.0;
            constexpr double d = 0.0;
            constexpr double f = 0.12;

            // Maximum angular velocity (rad/s)
            constexpr units::radians_per_second_t maxVelocity {10 * std::numbers::pi}; 

            // Maximum angular acceleration (rad/s^2)
            constexpr units::radians_per_second_squared_t maxAcceleration {10 * std::numbers::pi}; 

            constexpr units::radian_t tolerance = 0.2_deg;
        }

        // Drive motor PID values
        namespace drivePID 
        {
            constexpr double p = 1.5;
            constexpr double i = 0.0;
            constexpr double d = 0.0;
        }

        // Drive motor feedforward values
        namespace driveFF
        {
            constexpr auto s = 0.12_V; 
            constexpr auto v = 2.6_V * 1.0_s / 1.0_m;
            constexpr auto a = 0.0_V * 1.0_s * 1.0_s / 1.0_m;
        }

        // Heading lock PID values 
        namespace headingPID 
        {
            constexpr double p = 3.0;
            constexpr double i = 0.0;
            constexpr double d = 0.0;
            constexpr units::radians_per_second_t ff = 6_deg_per_s; // 0.1_rad_per_s;
            constexpr units::radian_t tolerance = 2.0_deg;
        }

        // Preference keys for testing
        namespace preferences
        {
            constexpr std::string_view driveP_Key = "Drive PID P";
            constexpr std::string_view driveI_Key = "Drive PID I";
            constexpr std::string_view driveD_Key = "Drive PID D";
            constexpr std::string_view driveFF_S_Key = "Drive FF S";
            constexpr std::string_view driveFF_V_Key = "Drive FF V";
            
            constexpr std::string_view turnP_Key = "Turn PID P";
            constexpr std::string_view turnI_Key = "Turn PID I";
            constexpr std::string_view turnD_Key = "Turn PID D";
            constexpr std::string_view turnV_Key = "Turn PID V";
            constexpr std::string_view turnA_Key = "Turn PID A";
            constexpr std::string_view turnF_Key = "Turn FF";

            constexpr std::string_view offsetFL_Key = "Front Left Offset";
            constexpr std::string_view offsetFR_Key = "Front Right Offset";
            constexpr std::string_view offsetBL_Key = "Back Left Offset";
            constexpr std::string_view offsetBR_Key = "Back Right Offset";
        }
    }
}