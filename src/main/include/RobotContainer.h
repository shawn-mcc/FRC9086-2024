#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/DriveSystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
    RobotContainer();

    // frc2::Command* GetAutonomousCommand();

private:
    // The driver's controller
    frc::XboxController m_primaryController{0};
    frc::XboxController m_secondaryController{1};

    // The robot's subsystems and commands are defined here...

    // The robot's subsystems
    DriveSystem m_drive;

    // The chooser for the autonomous routines
    frc::SendableChooser<frc2::Command*> m_chooser;

    void ConfigureBindings();
};