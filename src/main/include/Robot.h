
#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {

    SwerveModule swrv_frontLeft = SwerveModule(2, 1, 1.57);
    SwerveModule swrv_frontRight = SwerveModule(18, 19, 0);
    SwerveModule swrv_backLeft = SwerveModule(8, 7, 3.14);
    SwerveModule swrv_backRight = SwerveModule(10, 11, 4.71);


    frc::XboxController m_primaryController{0};
    frc::XboxController m_secondaryController{1};

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
};