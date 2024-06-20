// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Robot.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "subsystems/LauncherController.h"
#include "subsystems/ArmController.h"
#include "subsystems/FireControl.h"
#include "RobotUtils.h"
#include "frc/XboxController.h"
#include "frc/BuiltInAccelerometer.h"
#include "cameraserver/CameraServer.h"

void Robot::RobotInit() {
	camera1 = frc::CameraServer::StartAutomaticCapture();
	camera1.SetResolution(640, 480);
	camera1.SetFPS(5);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	//Moved this stuff here in hopes of fixxing lag. Don't think it will actually work, but RobotPeriodic
	//is called before any specific modes.
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    frc::SmartDashboard::PutNumber("AutonState", 11);
  //m_autonomousCommand = m_container.GetAutonomousCommand();

  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Schedule();
  // }
} //change

void Robot::AutonomousPeriodic() {
	//Shoot first? Pressed right up against Amp to start
	//arm.SetArmPosition((5 * M_PI) / 18); //This is based on Peterson Math !THIS IS NOT THE FINAL NUMBER AND DO NOT NOT NOT USE IT!
	//fireController.Fire(1, 0); //??? Not sure why speed / trajectory both needed? Also 100 might be too much
	//sleep(1); //Just to make sure we don't drive before note fully leaves
	// Leave zone
	/*swrv_frontLeft.SetState(1, 0 + 1.57);
	swrv_frontRight.SetState(-1, 0 + 0);
	swrv_backLeft.SetState(-1, 0 + 3.14);
	swrv_backRight.SetState(1, 0 + 4.71);*/
	//Stop leaving Zone
	//sleep(RobotUtil.CalcMoveDuration(6.5)); //6.5ft is width of start zone to get those 2 points
	/*
	sleep(0.5);
	swrv_frontLeft.SetState(0, 0 + 1.57);
	swrv_frontRight.SetState(0, 0 + 0);
	swrv_backLeft.SetState(0, 0 + 3.14);
	swrv_backRight.SetState(0, 0 + 4.71);
	//No more auto
	//sleep(15 - RobotUtil.CalcMoveDuration(6.5) - 1);
	sleep(13.5);
	*/

	int autonState = frc::SmartDashboard::GetNumber("AutonState", 11);

	switch (autonState) {
	case 11:
		if (arm.SetArmPosition(2.4)) {
			frc::SmartDashboard::PutNumber("AutonState", autonState - 1);
		} else {
			arm.SetArmPosition(2.4);
			//fireController.Spool(.5,0);
			fireController.StopAll();
			sleep(.5);
		}
		break;
	case 10:
	case 9:
	case 8:
	case 7:
		arm.SetArmPosition(2.35);
		fireController.Spool(1, 0);
		sleep(1);
		frc::SmartDashboard::PutNumber("AutonState", autonState - 1);
		break;
	case 6:
	case 5:
	case 4:
	case 3:
		arm.SetArmPosition(2.35);
		sleep(1);
		fireController.Fire(1, .1);
		frc::SmartDashboard::PutNumber("AutonState", autonState - 1);
		break;
	case 2:
		arm.SetArmPosition(0);
		fireController.StopAll();
		sleep(1);
		frc::SmartDashboard::PutNumber("AutonState", 1);

		swrv_frontLeft.SetState(0, 3.14 + 1.57);
		swrv_frontRight.SetState(0, 3.14 + 0);
		swrv_backLeft.SetState(0, 3.14 + 3.14);
		swrv_backRight.SetState(0, 3.14 + 4.71);

		break;
	case 1:
		/*
		if ((swrv_frontLeft.GetDistance(1) > 1) ||
		(swrv_frontRight.GetDistance(1) > 1) ||
		(swrv_backLeft.GetDistance(1) > 1) ||
		(swrv_backRight.GetDistance(1) > 1)) {
			frc::SmartDashboard::PutNumber("AutonState", 0);

		} else {
			swrv_frontLeft.SetState(1, 3.14 + 1.57);
			swrv_frontRight.SetState(-1, 3.14 + 0);
			swrv_backLeft.SetState(-1, 3.14 + 3.14);
			swrv_backRight.SetState(1, 3.14 + 4.71);

			}*/
		swrv_frontLeft.SetState(-.5, 3.14 + 1.57);
		swrv_frontRight.SetState(.5, 3.14 + 0);
		swrv_backLeft.SetState(.5, 3.14 + 3.14);
		swrv_backRight.SetState(-.5, 3.14 + 4.71);
		sleep(1);
		frc::SmartDashboard::PutNumber("AutonState", 0);
		break;
	default:

		swrv_frontLeft.SetState(0, 3.14 + 1.57);
		swrv_frontRight.SetState(0, 3.14 + 0);
		swrv_backLeft.SetState(0, 3.14 + 3.14);
		swrv_backRight.SetState(0, 3.14 + 4.71);

		break;

	}


}

void Robot::TeleopInit() {

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
	// Primary controller
	double x1Input = m_primaryController.GetLeftX();
	double y1Input = m_primaryController.GetLeftY();
	double x2Input = m_primaryController.GetRightX();
	double y2Input = m_primaryController.GetRightY();
	double forwardThrottle = m_primaryController.GetRawAxis(3);
	double backwardThrottle = m_primaryController.GetRawAxis(2);
	double rotate = m_primaryController.GetBButton();
	double lockSwerve = m_primaryController.GetXButton();

	// Left stick -1 - 1 -> 0 - 2pi
	// Secondary controller
	double x3Input = m_secondaryController.GetLeftX();
	double y3Input = m_secondaryController.GetLeftY();
	double x4Input = m_secondaryController.GetRightX();
	double y4Input = m_secondaryController.GetRightY();
	double fireTrigger = m_secondaryController.GetRawAxis(3);
	double spoolTrigger = m_secondaryController.GetRawAxis(2);
	bool intakeButton = m_secondaryController.GetAButton();
	bool armUpButton = m_secondaryController.GetYButton();

	// Roborio positions
	double xAccel = accelerometer.GetX();
	double yAccel = accelerometer.GetY();
	double zAccel = accelerometer.GetZ();

	// Get the yaw angle of the Roborio
	double yawAngle = RobotUtil.GetYawAngle(xAccel, yAccel, zAccel);

	//
	// |input| * pi

	// Output variables
	double steeringAngle, speed = 0, armAngle, launcherAngle;
	// 0 - 2pi

    // Get the triggers to make throttles
    speed = forwardThrottle - backwardThrottle;

    // Swerve control
    steeringAngle = atan2(x1Input, y1Input);
    if (x1Input > 0) {
    	steeringAngle = (2 * M_PI) + steeringAngle;
    }

    // Fix swerve values when at 0, 0
    if (x1Input == 0 && y1Input == 0) {
        speed = -speed;
    }

	// Arm Control
	armAngle = fabs(y3Input) * 2.5; // use fabs() for absolute value

    // Launcher
    /*if (arm.GetLArmPosition() < (M_PI / 4) || arm.GetLArmPosition() > ((3 * M_PI) / 4)) { // Allow launcher to fully exten when out of deadzone
		launcherAngle = fabs(y4Input) * 3; // use fabs() for absolute value
	} else { // Force launcher to not extend when in dead zone
		launcherAngle = (fabs(y4Input) * 1.5) + 3.14;
	} */

    // Engage all drive modules
    if (rotate) {
    	swrv_frontLeft.SetState(speed, M_PI / 4);
    	swrv_frontRight.SetState(speed, M_PI / 4);
    	swrv_backLeft.SetState(speed, M_PI / 4);
    	swrv_backRight.SetState(speed, M_PI / 4);
    } else if (lockSwerve){
    	swrv_frontLeft.SetState(speed, (2 * M_PI) - M_PI / 4);
    	swrv_frontRight.SetState(speed, (2 * M_PI) - M_PI / 4);
    	swrv_backLeft.SetState(speed, (2 * M_PI) - M_PI / 4);
    	swrv_backRight.SetState(speed, (2 * M_PI) - M_PI / 4);

	} else {
 	    swrv_frontLeft.SetState(speed + x2Input, steeringAngle + 1.57);
	    swrv_frontRight.SetState(-speed + x2Input, steeringAngle + 0);
	    swrv_backLeft.SetState(-speed - x2Input, steeringAngle + 3.14);
	    swrv_backRight.SetState(speed - x2Input, steeringAngle + 4.71);
    }


    // Engage arm modules
	//arm.SetArmPosition(armAngle); //2.38

    // Engage launcher modules
    //launcher.SetLauncherPosition(launcherAngle);

    // Engage fire control system modules

    if (fireTrigger > .1) {
        fireController.Fire(fireTrigger, y4Input); // Actuvate all
    } else if (spoolTrigger > .1) {
        fireController.Spool(spoolTrigger, y4Input); // Activate spool mode
    } else if (intakeButton) {
        fireController.Intake(); // Activate intake mode
    } else {
        fireController.StopAll(); // Stop
    }

	if (armUpButton) {
		arm.SetArmPosition(2.3);
	} else if (intakeButton){
		arm.SetArmPosition(0.1);
	} else {
		arm.SetArmPosition(armAngle);
	}

/*
    //Pressure stuff
    bool A_Button = m_secondaryController.GetBButton();
    if (A_Button){
        m_solenoid1A.Set(true);
        m_solenoid1B.Set(false);
        m_solenoid2A.Set(true);
        m_solenoid2B.Set(false);
        frc::SmartDashboard::PutBoolean("Solenoid1", m_solenoid1A.Get());
        frc::SmartDashboard::PutBoolean("Solenoid2", m_solenoid2A.Get());
    }
    else{
        m_solenoid1A.Set(false);
        m_solenoid1B.Set(true);
        m_solenoid2A.Set(false);
        m_solenoid2B.Set(true);
        frc::SmartDashboard::PutBoolean("Solenoid1", m_solenoid1A.Get());
        frc::SmartDashboard::PutBoolean("Solenoid2", m_solenoid2A.Get());
    }
*/
	//frc::SmartDashboard::PutNumber("ARMLEFTPOSITION", leftACME.GetPosition());
	//frc::SmartDashboard::PutNumber("ARMRIGHTPOSITION", rightACME.GetPosition());

    // Display data
    frc::SmartDashboard::PutNumber("Roborio Angle", yawAngle);
    frc::SmartDashboard::PutNumber("Roborio X", xAccel);
    frc::SmartDashboard::PutNumber("Roborio Y", yAccel);
    frc::SmartDashboard::PutNumber("Roborio Z", zAccel);

    frc::SmartDashboard::PutNumber("Steering Angle", steeringAngle);
    frc::SmartDashboard::PutNumber("SWRVFLENC", swrv_frontLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVFRENC", swrv_frontRight.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRLENC", swrv_backLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRRENC", swrv_backRight.GetPosition());

	frc::SmartDashboard::PutNumber("SWRVFRROT", swrv_frontRight.GetDistance(1));
	frc::SmartDashboard::PutNumber("SWRVFLROT", swrv_frontLeft.GetDistance(1));
	frc::SmartDashboard::PutNumber("SWRVBRROT", swrv_backRight.GetDistance(1));
	frc::SmartDashboard::PutNumber("SWRVBLROT", swrv_backLeft.GetDistance(1));

	frc::SmartDashboard::PutNumber("Arm Angle", armAngle);
    frc::SmartDashboard::PutNumber("ARMRENC", arm.GetRArmPosition());
    frc::SmartDashboard::PutNumber("ARMLENC", arm.GetLArmPosition());

	//frc::SmartDashboard::PutNumber("Launcher Angle", launcherAngle);
    //frc::SmartDashboard::PutNumber("LAUNCHERENC", launcher.GetLauncherPosition());

}
/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
	arm.SetArmPosition(0);
/* Jetson is broken, can use when we have fixed one
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto smartDashboard = inst.GetTable("SmartDashboard");
	double command = smartDashboard->GetNumber("autoAim", 0);
	frc::SmartDashboard::PutNumber("Data recieved", command);
	frc::SmartDashboard::PutString("yes", typeid(command).name());
	// right
	if(command == 1){
		swrv_frontLeft.SetState(-0.035, M_PI / 4);
		swrv_frontRight.SetState(-0.035, M_PI / 4);
		swrv_backLeft.SetState(-0.035, M_PI / 4);
		swrv_backRight.SetState(-0.035, M_PI / 4);
	}
	// Left
	if(command == 2){
		swrv_frontLeft.SetState(0.035, M_PI / 4);
		swrv_frontRight.SetState(0.035, M_PI / 4);
		swrv_backLeft.SetState(0.035, M_PI / 4);
		swrv_backRight.SetState(0.035, M_PI / 4);
	}
	// Up
	if(command == 3){
		swrv_frontLeft.SetState(0, M_PI / 4);
		swrv_frontRight.SetState(0, M_PI / 4);
		swrv_backLeft.SetState(0, M_PI / 4);
		swrv_backRight.SetState(0, M_PI / 4);
	}
	// Down
	if(command == 4){
		swrv_frontLeft.SetState(0, M_PI / 4);
		swrv_frontRight.SetState(0, M_PI / 4);
		swrv_backLeft.SetState(0, M_PI / 4);
		swrv_backRight.SetState(0, M_PI / 4);
	}
	if(command == 5){
		swrv_frontLeft.SetState(0, M_PI / 4);
		swrv_frontRight.SetState(0, M_PI / 4);
		swrv_backLeft.SetState(0, M_PI / 4);
		swrv_backRight.SetState(0, M_PI / 4);
	}
*/

/*
	swrv_frontLeft.SetState(0, 0.001 + 1.57 + 3.14);
	swrv_frontRight.SetState(0, 0.001 + 0);
	swrv_backLeft.SetState(0, 0.001 + 3.14);
	swrv_backRight.SetState(0, 0.001 + 4.71 + 3.14);

	frc::SmartDashboard::PutBoolean("SWRVFRAUT", swrv_frontLeft.SetDistanceState(1, 0.001 + 1.57 + 3.14));
	frc::SmartDashboard::PutBoolean("SWRVFLAUT", swrv_frontRight.SetDistanceState(1, 0.001 + 0));
	frc::SmartDashboard::PutBoolean("SWRVBRAUT", swrv_backLeft.SetDistanceState(1, 0.001 + 3.14));
	frc::SmartDashboard::PutBoolean("SWRVBLAUT", swrv_backRight.SetDistanceState(1, 0.001 + 4.71 + 3.14));

	frc::SmartDashboard::PutNumber("SWRVFRROT", swrv_frontRight.GetDistance());
	frc::SmartDashboard::PutNumber("SWRVFLROT", swrv_frontLeft.GetDistance());
	frc::SmartDashboard::PutNumber("SWRVBRROT", swrv_backRight.GetDistance());
	frc::SmartDashboard::PutNumber("SWRVBLROT", swrv_backLeft.GetDistance());
*/
}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
