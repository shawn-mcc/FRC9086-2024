// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"
#include "subsystems/ArmController.h"
#include "frc/XboxController.h"

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

    double x1Input = m_primaryController.GetLeftX();
    double y1Input = m_primaryController.GetLeftY();
    double x2Input = m_primaryController.GetRightX();
    double y2Input = m_primaryController.GetRightY();
    double x3Input = m_secondaryController.GetLeftY();
    double y3Input = m_secondaryController.GetRightY();

    // Swerve control
    double steeringAngle = atan2(y1Input, x1Input);

    if (steeringAngle < 0) {
        steeringAngle = -steeringAngle;
    }

    swrv_frontLeft.SetState((x2Input + y2Input),steeringAngle);
    swrv_frontRight.SetState((x2Input - y2Input), steeringAngle);
    swrv_backLeft.SetState((x2Input + y2Input), steeringAngle);
    swrv_backRight.SetState((x2Input - y2Input), steeringAngle);

    // Display angle
    frc::SmartDashboard::PutNumber("Steering angle", steeringAngle);
    frc::SmartDashboard::PutNumber("SWRVFLENC", swrv_frontLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVFRENC", swrv_frontRight.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRLENC", swrv_backLeft.GetPosition());
    frc::SmartDashboard::PutNumber("SWRVRRENC", swrv_backRight.GetPosition());


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
