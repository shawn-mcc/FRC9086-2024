// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "subsystems/LauncherController.h"
#include "subsystems/ArmController.h"
#include "subsystems/FireControl.h"
#include "RobotUtils.h"
#include "frc/XboxController.h"
#include "frc/BuiltInAccelerometer.h"

void Robot::RobotInit() {
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

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
	// 0 - 2pi

    // Primary controller
    double x1Input = m_primaryController.GetLeftX();
    double y1Input = m_primaryController.GetLeftY();
    double x2Input = m_primaryController.GetRightX();
    double y2Input = m_primaryController.GetRightY();
    double forwardThrottle = m_primaryController.GetRawAxis(3);
    double backwardThrottle = m_primaryController.GetRawAxis(2);

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
	armAngle = fabs(y3Input) * M_PI; // use fabs() for absolute value

    // Launcher
	launcherAngle = fabs(y4Input) * 3; // use fabs() for absolute value

    // Engage all drive modules
    swrv_frontLeft.SetState(speed + x2Input, steeringAngle + 1.57);
    swrv_frontRight.SetState(-speed + x2Input, steeringAngle + 0);
    swrv_backLeft.SetState(-speed - x2Input, steeringAngle + 3.14);
    swrv_backRight.SetState(speed - x2Input, steeringAngle + 4.71);


    // Engage arm modules
	arm.SetArmPosition(armAngle);

    // Engage launcher modules
    //launcher.SetLauncherPosition(0);

    // Engage fire control system modules
    if (fireTrigger > .1) {
        fireController.Fire(fireTrigger); // Actuvate all
    } else if (spoolTrigger > .1) {
        fireController.Spool(spoolTrigger); // Activate spool mode
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

	frc::SmartDashboard::PutNumber("Launcher Angle", launcherAngle);
    frc::SmartDashboard::PutNumber("LAUNCHERENC", launcher.GetLauncherPosition());

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
