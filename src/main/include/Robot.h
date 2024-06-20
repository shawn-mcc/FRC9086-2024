
#pragma once

#include <frc/TimedRobot.h>
#include <frc/PneumaticHub.h>
#include "subsystems/SwerveModule.h"
#include "subsystems/LauncherController.h"
#include "subsystems/ArmController.h"
#include "subsystems/FireControl.h"
#include "RobotUtils.h"
#include "frc/XboxController.h"
#include "frc/BuiltInAccelerometer.h"
#include <frc/PneumaticHub.h>
#include <frc/Solenoid.h>
#include "cameraserver/CameraServer.h"


class Robot : public frc::TimedRobot {

    RobotUtils RobotUtil;

    frc::BuiltInAccelerometer accelerometer;

    // Robot components
    SwerveModule swrv_frontLeft = SwerveModule(2, 1);
    SwerveModule swrv_frontRight = SwerveModule(18, 19);
    SwerveModule swrv_backLeft = SwerveModule(8, 7);
    SwerveModule swrv_backRight = SwerveModule(10, 11);

	ArmController arm = ArmController(3, 13);

    //LauncherController launcher = LauncherController(4, 16);

    FireControl fireController = FireControl(14, 5, 15);


    /*frc::PneumaticHub m_PH{9};

    frc::Solenoid m_solenoid1A = m_PH.MakeSolenoid(8);
    frc::Solenoid m_solenoid1B = m_PH.MakeSolenoid(9);

    frc::Solenoid m_solenoid2A = m_PH.MakeSolenoid(14);
    frc::Solenoid m_solenoid2B = m_PH.MakeSolenoid(15);*/

    // Controls
    frc::XboxController m_primaryController{0};
    frc::XboxController m_secondaryController{1};

	cs::UsbCamera camera1;

public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    // Have it null by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    frc::PneumaticHub m_ph{9};
};