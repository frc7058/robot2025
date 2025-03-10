#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>


//#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
//#include <pathplanner/lib/util/PIDConstants.h>
//#include <pathplanner/lib/util/ReplanningConfig.h>
#include <units/time.h>
#include <string>
#include <memory>
#include "constants/DriveConstants.h"

namespace constants 
{
    namespace autonomous 
    {
        const std::string noneAuto = "None";
        const std::string defaultAuto = noneAuto;
        const std::string oneNoteAuto = "One Note";

        const std::string autoNames[] = {
            noneAuto,
        };

        const pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

        const auto pathFollowerConfig = std::make_shared<pathplanner::PPHolonomicDriveController>(
            // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0),

            // Rotation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0)
        );

        constexpr units::second_t intakeTimeLimit = 3.0_s;
    }
}