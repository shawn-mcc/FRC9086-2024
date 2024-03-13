/*
 * Project: 9086-2024
 * File: SwerveModule.cpp
 * Description: Create swerve modules to be used in other files, modules built using REV Robotics CAN SparkMax Motor Controllers.
 */

#include "subsystems/SwerveModule.h"
#include "RobotUtils.h"
#include "cmath"
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

// Create each swerve module
SwerveModule::SwerveModule(const int driveID, const int steerID):
    m_drive(driveID, rev::CANSparkMax::MotorType::kBrushless),
    m_steer(steerID, rev::CANSparkMax::MotorType::kBrushless) {

    // Reset to the defaults to be safe
    m_drive.RestoreFactoryDefaults();
    m_steer.RestoreFactoryDefaults();

    // Set motor current limits to prevent them from cooking
    m_drive.SetSmartCurrentLimit(40);
    m_steer.SetSmartCurrentLimit(20);

    // Set modes to brake
    m_drive.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_steer.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Set encoder
    m_steerPID.SetFeedbackDevice(m_steerEncoder);

    // Establish PID controls to keep the motors in proper positions
    // These can be changed if the need arises
    m_drivePID.SetP(0.1);
    m_drivePID.SetI(1e-4);
    m_drivePID.SetD(1);
    m_drivePID.SetIZone(0);
    m_drivePID.SetFF(0);
    m_drivePID.SetOutputRange(-1, 1);

    m_steerEncoder.SetPositionConversionFactor(2 * M_PI);
    m_steerEncoder.SetInverted(true);

    m_steerPID.SetP(0.1);
    m_steerPID.SetI(1e-4);
    m_steerPID.SetD(1);
    m_steerPID.SetIZone(0);
    m_steerPID.SetFF(0);
    m_steerPID.SetOutputRange(-1, 1);

    // Make it so that PID understands limits and can work around them
    m_steerPID.SetPositionPIDWrappingEnabled(true);
    
    m_steerPID.SetPositionPIDWrappingMinInput(0);
    m_steerPID.SetPositionPIDWrappingMaxInput(2 * M_PI);

    // Burn flash to save to memory
    m_drive.BurnFlash();
    m_steer.BurnFlash();
}

// Call this to get the position value of the steering system
double SwerveModule::GetPosition() {
    double position = m_steerEncoder.GetPosition();
    return position;
}

// Call this to set module to a position (in radians) and drive (in percentage 0-1)
void SwerveModule::SetState(double driveSpeed, double steerPosition) {

    // Initialize variables
    double currentPosition, correctedPosition = steerPosition;//RobotUtil.GetCorrectedAngle(steerPosition + originalAngle);
    frc::SmartDashboard::PutNumber("SWRV", correctedPosition);

    // Set steering position to the correct position using PID, steers with radians
    m_steerPID.SetReference(correctedPosition, rev::CANSparkMax::ControlType::kPosition);

    // Get current position
    currentPosition = RobotUtil.GetCorrectedAngle(m_steerEncoder.GetPosition());

    // Drive
    m_drive.Set(driveSpeed);
}