#include "commands/DriveCommand.h"
#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
#include "lib/FieldUtil.h"
#include "lib/Util.h"
#include <frc/MathUtil.h>
#include <units/math.h>

DriveCommand::DriveCommand(DriveBase* driveBase, frc::XboxController& driveController)
    : m_driveBase(driveBase), m_driveController(driveController)
{
    AddRequirements(m_driveBase);
}

void DriveCommand::Execute()
{
    double leftX = frc::ApplyDeadband(m_driveController.GetLeftX(), constants::controls::joystickDeadband);
    double leftY = frc::ApplyDeadband(m_driveController.GetLeftY(), constants::controls::joystickDeadband);
    double rightX = frc::ApplyDeadband(m_driveController.GetRightX(), constants::controls::joystickDeadband);

    units::meters_per_second_t velocityX = util::sign(leftY) * -std::abs(std::pow(leftY, 2.0)) * constants::drive::maxDriveVelocity;
    units::meters_per_second_t velocityY = util::sign(leftX) * -std::abs(std::pow(leftX, 2.0)) * constants::drive::maxDriveVelocity;
    units::radians_per_second_t angularVelocity = util::sign(rightX) * -std::abs(std::pow(rightX, 3.0)) * constants::drive::maxAngularVelocity;

    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

    if(m_driveController.GetLeftTriggerAxis() > constants::controls::axisDeadband)
    {
        // Note target lock

        std::optional<units::radian_t> angleToTarget;
        if(angleToTarget)
        {
            units::radian_t heading = m_driveBase->GetHeading();
            units::radian_t targetHeading = heading + angleToTarget.value();
            fmt::print("Angle to object: {}, Target: {}\n", angleToTarget.value(), targetHeading);

            m_driveBase->TrackHeading(targetHeading);
            // velocityY = 0_mps;
        }
        else 
        {
            fmt::print("No target found\n");
        }

        angularVelocity = 0_rad_per_s;

        // velocityX = util::sign(leftY) * -std::abs(std::pow(leftY, 2.0)) * constants::drive::slowMaxDriveVelocity;
        // velocityY = util::sign(leftX) * -std::abs(std::pow(leftX, 2.0)) * constants::drive::slowMaxDriveVelocity;
        // angularVelocity = util::sign(rightX) * -std::abs(std::pow(rightX, 3.0)) * constants::drive::slowMaxAngularVelocity;

    }
    else if(m_driveController.GetRightTriggerAxis() > constants::controls::axisDeadband && alliance.has_value())
    {
        // Speaker/amp target lock

        // frc::Translation2d currentPosition = m_driveBase.GetPose().Translation();

        // frc::Translation2d speakerPosition = GetSpeakerPosition(alliance.value());
        // frc::Translation2d ampPosition = GetSpeakerPosition(alliance.value());

        // frc::Translation2d targetPosition = currentPosition.Nearest({ speakerPosition, ampPosition });
        
        // units::radian_t angleToTarget = units::math::atan2(
        //     targetPosition.Y() - currentPosition.Y(),
        //     targetPosition.X() - currentPosition.X());

        // m_driveBase.TrackObject(angleToTarget);
    }
    else 
    {
        m_driveBase->DisableTracking();
    }

    //m_driveBase->Drive(velocityX, velocityY, angularVelocity, !m_driveBase->IsTrackingEnabled());
    m_driveBase->Drive(velocityX, velocityY, angularVelocity, true);
}