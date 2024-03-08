/*
 * Project: 9086-2024
 * File: DriveSystems.cpp
 * Description: Using all of the swerve modules together along with any other drive components
 */

#include "subsystems/SwerveModule.h"
#include "math.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/DriveSystem.h"

//Create each module separately
DriveSystem::DriveSystem():
    //SwerveModule.cpp says that each module is defined as {driveID, steerID, originalAngle}
    swrv_frontLeft{2, 1, 1.27},// pi/2 is 1.57, not 1.27???
    swrv_frontRight{18, 19, 0},
    swrv_backLeft{8, 7, 3.14},
    swrv_backRight{10, 11, 4.41}, // 4.71 is 3pi/2????
    arm_controller{3, 17}, // Placeholder values
    launcher_controller{4, 16} // Placeholder values
    {}

// This executes frequently
void DriveSystem::Periodic() {
}

//Teleoperated drive mode, only for interacting with swerve modules
void DriveSystem::TeleopDrive(double x1Input, double y1Input, double x2Input, double y2Input, double armInput, double launcherInput) {

    int count = 0;

    while (count <= 10000) {
        swrv_frontLeft.SetState(0, 0);
        swrv_frontRight.SetState(0, 0);
        swrv_backLeft.SetState(0, 0);
        swrv_backRight.SetState(0, 0);
        frc::SmartDashboard::PutNumber("count", count);
        count++;
        sleep(.01);
    }

    sleep(5);

    count = 0;

    while (count <= 10000) {
        swrv_frontLeft.SetState(0, M_PI/2);
        swrv_frontRight.SetState(0, M_PI/2);
        swrv_backLeft.SetState(0, M_PI/2);
        swrv_backRight.SetState(0, M_PI/2);
        frc::SmartDashboard::PutNumber("count", count);
        count++;
        sleep(.01);
    }

    sleep(5);
    count = 0;


    while (count <= 10000) {
        swrv_frontLeft.SetState(0, M_PI);
        swrv_frontRight.SetState(0, M_PI);
        swrv_backLeft.SetState(0, M_PI);
        swrv_backRight.SetState(0, M_PI);
        frc::SmartDashboard::PutNumber("count", count);
        count++;
        sleep(.01);
    }

    sleep(5);
    count = 0;


    while (count <= 10000) {
        swrv_frontLeft.SetState(0, M_PI/4);
        swrv_frontRight.SetState(0, M_PI/4);
        swrv_backLeft.SetState(0, M_PI/4);
        swrv_backRight.SetState(0, M_PI/4);
        frc::SmartDashboard::PutNumber("count", count);
        count++;
        sleep(.01);
    }

    sleep(5);



//Initilize variables
    double steeringAngle, armAngle, launcherAngle;

    // Swerve control
    steeringAngle = atan2(y1Input, x1Input);

    if (steeringAngle < 0) {
        steeringAngle = -steeringAngle;
    }

    swrv_frontLeft.SetState((x2Input + y2Input), steeringAngle);
    swrv_frontRight.SetState((x2Input - y2Input), steeringAngle);
    swrv_backLeft.SetState((x2Input + y2Input), steeringAngle);
    swrv_backRight.SetState((x2Input - y2Input), steeringAngle);

    // Display angle
    frc::SmartDashboard::PutNumber("Steering angle", steeringAngle);
    frc::SmartDashboard::PutNumber("SWRVFLENC", swrv_frontLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVFRENC", swrv_frontRight.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRLENC", swrv_backLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRRENC", swrv_backRight.GetPosition());

    // Arm control
    armAngle = asin(armInput);
    arm_controller.SetPosition(armAngle);

    // Launcher control
    launcherAngle = asin(launcherInput);
    launcher_controller.SetPosition(launcherAngle);

}

//Autonomous drive code, only for interacting with swerve modules
void DriveSystem::AutonDrive(){
    
}

//Test drive code, only for interacting with swerve modules
void DriveSystem::TestDrive(){

}