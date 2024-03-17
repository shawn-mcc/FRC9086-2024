// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "subsystems/LauncherController.h"
#include "subsystems/ArmController.h"
#include "subsystems/FireControl.h"
#include "subsystems/rat.cpp"
#include "RobotUtils.h"
#include "frc/XboxController.h"
#include "frc/BuiltInAccelerometer.h"
#include "cameraserver/CameraServer.h"

void Robot::RobotInit() {
	camera1 = frc::CameraServer::StartAutomaticCapture();
	camera1.SetResolution(320,240);
	camera1.SetFPS(10);
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
	// Primary controller
	double x1Input = m_primaryController.GetLeftX();
	double y1Input = m_primaryController.GetLeftY();
	double x2Input = m_primaryController.GetRightX();
	double y2Input = m_primaryController.GetRightY();
	double forwardThrottle = m_primaryController.GetRawAxis(3);
	double backwardThrottle = m_primaryController.GetRawAxis(2);
	double rotate = m_primaryController.GetBButton();

	// Left stick -1 - 1 -> 0 - 2pi
	// Secondary controller
	double x3Input = m_secondaryController.GetLeftX();
	double y3Input = m_secondaryController.GetLeftY();
	double x4Input = m_secondaryController.GetRightX();
	double y4Input = m_secondaryController.GetRightY();
	double fireTrigger = m_secondaryController.GetRawAxis(3);
	double spoolTrigger = m_secondaryController.GetRawAxis(2);
	bool intakeButton = m_secondaryController.GetAButton();

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
  //m_autonomousCommand = m_container.GetAutonomousCommand();

  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Schedule();
  // }
} //change

void Robot::AutonomousPeriodic() {
	//Shoot first? Pressed right up against Amp to start
	arm.SetArmPosition(63.3); //This is based on Peterson Math
	fireController.Fire(100/100, 0); //??? Not sure why speed / trajectory both needed? Also 100 might be too much
	sleep(1) //Just to make sure we don't drive before note fully leaves
	// Leave zone
	swrv_frontLeft.SetState(1, 0 + 1.57);
	swrv_frontRight.SetState(-1, 0 + 0);
	swrv_backLeft.SetState(-1, 0 + 3.14);
	swrv_backRight.SetState(1, 0 + 4.71);
	//Stop leaving Zone
	sleep(CalcMoveDuration(6.5)); //6.5ft is width of start zone to get those 2 points
	swrv_frontLeft.SetState(0, 0 + 1.57);
	swrv_frontRight.SetState(0, 0 + 0);
	swrv_backLeft.SetState(0, 0 + 3.14);
	swrv_backRight.SetState(0, 0 + 4.71);
	//No more auto
	sleep(15 - CalcMoveDuration(6.5) - 1);

}

void Robot::TeleopInit() {

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
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
	armAngle = fabs(y3Input) * 2.4; // use fabs() for absolute value

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
    } else {
	    swrv_frontLeft.SetState(speed + x2Input, steeringAngle + 1.57);
	    swrv_frontRight.SetState(-speed + x2Input, steeringAngle + 0);
	    swrv_backLeft.SetState(-speed - x2Input, steeringAngle + 3.14);
	    swrv_backRight.SetState(speed - x2Input, steeringAngle + 4.71);
    }


    // Engage arm modules
	arm.SetArmPosition(armAngle);

    // Engage launcher modules
    //launcher.SetLauncherPosition(launcherAngle);

    // Engage fire control system modules
    if (fireTrigger > .1) {
        fireController.Fire(fireTrigger, y4Input / 2.0); // Actuvate all
    } else if (spoolTrigger > .1) {
        fireController.Spool(spoolTrigger, y4Input / 2.0); // Activate spool mode
    } else if (intakeButton) {
        fireController.Intake(); // Activate intake mode
    } else {
        fireController.StopAll(); // Stop
    }

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

	frc::SmartDashboard::PutNumber("Arm Angle", armAngle);
    frc::SmartDashboard::PutNumber("ARMRENC", arm.GetRArmPosition());
    frc::SmartDashboard::PutNumber("ARMLENC", arm.GetLArmPosition());

	//frc::SmartDashboard::PutNumber("Launcher Angle", launcherAngle);
    //frc::SmartDashboard::PutNumber("LAUNCHERENC", launcher.GetLauncherPosition());

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

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
