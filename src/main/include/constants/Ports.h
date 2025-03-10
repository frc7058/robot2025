#pragma once

namespace ports 
{
    namespace drive 
    {
        namespace driveMotorCAN 
        {
            constexpr int frontLeft = 6;
            constexpr int frontRight = 3;
            constexpr int backLeft = 7;
            constexpr int backRight = 5;
        }

        namespace turnMotorCAN 
        {
            constexpr int frontLeft = 9;
            constexpr int frontRight = 2;
            constexpr int backLeft = 8;
            constexpr int backRight = 4;
        }

        namespace CANCoder 
        {
            constexpr int frontLeft = 10;
            constexpr int frontRight = 11;
            constexpr int backLeft = 12;
            constexpr int backRight = 13;
        }
    }

    namespace intake 
    {
        constexpr int intakeMotorCAN = 14;
    }

    namespace dio 
    {
        ////
    }

    namespace pwm 
    {
        ////
    }

    namespace pdh 
    {
        constexpr int pdhCAN = 1;
    }
}