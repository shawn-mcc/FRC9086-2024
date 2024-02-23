// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "subsystems/DriveSystem.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  DriveSubsystem m_drive;

  // Configure the button bindings
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.TeleopDrive(
        m_driverController.GetLeftX();,
        m_driverController.GetLeftY();,
        m_driverController.GetRightX();,
        m_driverController.GetRightY();
      )
    }
  ))
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
}
