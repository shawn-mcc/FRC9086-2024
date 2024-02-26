/*
 * Project: 9086SWRV
 * File: DriveSystems.cpp
 * Description: Using all of the swerve modules together, ONLY SWERVE DRIVE INTERACTIONS
 */

#include "subsystems/SwerveModule.h"
#include "math.h"

#include "subsystems/DriveSystem.h"

//Create each module separately
DriveSystem::DriveSystem():
    //SwerveModule.cpp says that each module is defined as {driveID, steerID, originalAngle}
    swrv_frontLeft(3, 8, 1.27),
    swrv_frontRight(7, 6, 0),
    swrv_backLeft(5, 4, 3.14),
    swrv_backRight(11, 10, 4.41) {}


//Teleoperated drive mode, only for interacting with swerve modules
void DriveSystem::TeleopDrive(double x1Input, double y1Input, double x2Input, double y2Input) {
    //Initilize variables
    double steeringAngle;

    steeringAngle = atan2(y1Input, x1Input);

    swrv_frontLeft.SetState((x2Input + y2Input), steeringAngle);
    swrv_frontRight.SetState((x2Input - y2Input), steeringAngle);
    swrv_backLeft.SetState((x2Input + y2Input), steeringAngle);
    swrv_backRight.SetState((x2Input - y2Input), steeringAngle);

}

//Autonomous drive code, only for interacting with swerve modules
void DriveSystem::AutonDrive(){
    
}

//Test drive code, only for interacting with swerve modules
void DriveSystem::TestDrive(){

}