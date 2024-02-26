/*
 * Project: 9086SWRV
 * File: RobotUtils.cpp
 * Description: Useful robot code that can be called anywhere in the code as long as RobotUtils.h is included
 */

#include "RobotUtils.h"
#include "cmath"

// Call this to get the opposite angle of something
double RobotUtils::GetOppositeAngle(double angle) {
    angle += M_PI;

    //Check to make sure that the opposite angle is still within 2pi
    if (angle > (M_PI * 2)){

        angle -= (M_PI * 2);

    }

    return angle;
}

// Call this to get an angle between 0 and 2pi
double RobotUtils::GetCorrectedAngle(double angle) {

    while (angle > (M_PI * 2) || angle < 0) {

        if (angle > (M_PI * 2)) { // Angle is over 2pi

            angle -= (M_PI * 2);

        } else if (angle < 0) { // Angle is less than 0

            angle += (M_PI * 2);

        }
    }

    return angle;
}