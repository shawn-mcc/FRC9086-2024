/*
 * Project: 9086SWRV
 * File: DriveSystems.cpp
 * Description: Using all of the swerve modules together, ONLY SWERVE DRIVE INTERACTIONS
 */

#include "SwerveModule.h"
#include "math.h"

//Create each module separately
DriveSystem::DriveSystem():
    //SwerveModule.cpp says that each module is defined as {driveID, steerID, originalAngle}
    swrv_frontLeft{, , 1.27},
    swrv_frontRight{, , 0},
    swrv_backLeft{, , 3.14},
    swrv_backRight{, , 4.41} {}

//Teleoperated drive mode, only for interacting with swerve modules
void DriveSystem::TeleopDrive(double x1Input, double y1Input, double x2Input, double y2Input) {
    //Initilize variables
    double steeringAngle, driveOutput;

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