/*
 * Project: 9086SWRV
 * File: SwerveModule.cpp
 * Description: Create swerve modules to be used in other files, modules built using REV Robotics CAN SparkMax Motor Controllers.
 */

#include "subsystems/SwerveModule.h"
#include "RobotUtils.h"
#include "cmath"
#include "rev/CANSparkMax.h"

// Create each swerve module
SwerveModule::SwerveModule(const int driveID, const int steerID, const double originalAngle): 
    m_drive(driveID, rev::CANSparkMax::MotorType::kBrushless),
    m_steer(steerID, rev::CANSparkMax::MotorType::kBrushless) {

    // Reset to the defaults to be safe
    m_drive.RestoreFactoryDefaults();
    m_steer.RestoreFactoryDefaults();
    
    // Create PID controllers
    rev::SparkPIDController m_drivePID = m_drive.getPIDController();
    rev::SparkPIDController m_steerPID = m_steer.getPIDController();

    // Create encoder
    rev::SparkAbsoluteEncoder m_steerEncoder = m_drive.GetEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Set motor current limits to prevent them from cooking
    m_drive.SetSmartCurrentLimit(40);
    m_steer.SetSmartCurrentLimit(20);

    // Set modes to brake
    m_drive.SetIdleMode(kBrake);
    m_steer.SetIdleMode(kBrake);

    // Establish PID controls to keep the motors in proper positions
    // These can be changed if the need arises
    m_drivePID.SetP(0.1);
    m_drivePID.SetI(1e-4);
    m_drivePID.SetD(1);
    m_drivePID.SetFF(0);
    m_drivePID.SetOutputRange(1, -1);
    
    m_steerPID.SetP(0.1);
    m_steerPID.SetI(1e-4);
    m_steerPID.SetD(1);
    m_steerPID.SetFF(0);
    m_steerPID.SetOutputRange(0, 6.28);

    // Make it so that PID understands limits and can work around them
    m_steerPID.SetPositionPIDWrappingEnabled(true);
    m_steerPID.SetFeedBackDevice(m_driveEncoder);
    
    m_steerPID.SetPositionPIDWrappingMinInput(0);
    m_steerPID.SetPositionPIDWrappingMaxInput(6.28);

    // Burn flash to save to memory
    m_drive.BurnFlash();
    m_steer.BurnFlash();
}

// Call this to get the position value of the steering system
void SwerveModule::GetPosition() {
    double position = m_steerEncoder.GetPosition();
    return(position);
}

// Call this to set module to a position (in radians) and drive (in percentage 0-1)
void SwerveModule::SetState(double driveSpeed, double steerPosition) {

    // Initialize varaibles
    double currentPosition, errorMargin = .01;

    // Set steering position to the correct position using PID, steers with radians
    m_steer.SetReference(steerPosition, rev::CANSparkMax::ControlType::kPosition);

    // Get current position
    currentPosition = m_steer.GetPosition() + originalAngle;

    if (currentPosition * (1 - errorMargin) > steerPosition && currentPosition * (1 + errorMargin) < steerPosition) {
        // Drive
        m_drive.set(driveSpeed);
    }
}