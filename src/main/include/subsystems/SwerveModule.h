/*
 * Project: 9086-2024
 * File: SwerveModule.h
 * Description: Create swerve modules to be used in other files, modules built using REV Robotics CAN SparkMax Motor Controllers.
 */

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include "RobotUtils.h"

class SwerveModule {
public:

    SwerveModule(int driveID, int steerID,
                    double originalAngle);

    double GetPosition();

    void SetState(double driveSpeed, double steerPosition);


private:
    // Get offset
    double originalAngle;

    RobotUtils RobotUtil;

    // Get motors
    rev::CANSparkMax m_drive;
    rev::CANSparkMax m_steer;

    // Create PID controllers
    rev::SparkPIDController m_drivePID = m_drive.GetPIDController();
    rev::SparkPIDController m_steerPID = m_steer.GetPIDController();

    // Create encoder
    rev::SparkAbsoluteEncoder m_steerEncoder = m_drive.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

};