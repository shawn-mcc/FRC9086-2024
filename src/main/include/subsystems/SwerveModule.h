
#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

class SwerveModule {
public:
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    SwerveModule(int driveID, int steerID,
                    double originalAngle);

    double GetPosition();

    void SetState(double driveSpeed, double steerPosition);


private:
    // Get offset
    double originalAngle;

    // Get motors
    rev::CANSparkMax m_drive;
    rev::CANSparkMax m_steer;

    // Create PID controllers
    rev::SparkPIDController m_drivePID = m_drive.GetPIDController();
    rev::SparkPIDController m_steerPID = m_steer.GetPIDController();

    // Create encoder
    rev::SparkAbsoluteEncoder m_steerEncoder = m_drive.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

};