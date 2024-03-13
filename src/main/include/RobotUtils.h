/*
 * Project: 9086-2024
 * File: RobotUtils.h
 * Description: Assortment of commands that can be used in many places
 */

#pragma once

#include "cmath"

class RobotUtils {
public:

    // Call this to get the opposite angle of something
    double GetOppositeAngle(double angle);

    // Call this to get an angle between 0 and 2pi
    double GetCorrectedAngle(double angle);

    // Call this to get the robot's yaw angle
    double GetYawAngle(double aX, double aY, double aZ);

private:
};
