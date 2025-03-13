#include "commands/DriveCommand.h"
#include "constants/GeneralConstants.h"
#include "constants/DriveConstants.h"
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

    //m_driveBase->Drive(velocityX, velocityY, angularVelocity, !m_driveBase->IsTrackingEnabled());
    m_driveBase->Drive(velocityX, velocityY, angularVelocity, true);
}