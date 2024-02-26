/*
* Project: LauncherController
 * File: LauncherController.cpp
 * Description: Take angle as an input then move and hold launcher at that angle.
 */
#include <rev/CANSparkMax.h>

//PID coefficients
double kP = 0.1,
       kI = 1e-4, kD = 1,
       kIz = 0, kFF = 0,
       kMaxOutput = 1,
       kMinOutput = -1;

LauncherController::LauncherController(rev::CANSparkMax leftLCM, rev::CANSparkMax rightLCM):
    // Restore factory defaults because all the examples do it
    leftLCM.RestoreFactoryDefaults();
    rightLCM.RestoreFactoryDefaults();

    // Create encoders
    rev::SparkMaxAlternateEncoder leftLCME = leftLCM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192);
    rev::SparkMaxAlternateEncoder rightLCME = rightLCM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192);

    // Create PID controllers
    rev::SparkPIDController leftLCM_PID = leftLCM.getPIDController();
    rev::SparkPIDController rightLCM_PID = rightLCM.getPIDController();

    // Set PID coefficients
    leftLCM_PID.SetP(kP);
    leftLCM_PID.SetI(kI);
    leftLCM_PID.SetD(kD);
    leftLCM_PID.SetIZone(kIz);
    leftLCM_PID.SetFF(kFF);
    leftLCM_PID.SetOutputRange(kMinOutput, kMaxOutput);

    rightLCM_PID.SetP(kP);
    rightLCM_PID.SetI(kI);
    rightLCM_PID.SetD(kD);
    rightLCM_PID.SetIZone(kIz);
    rightLCM_PID.SetFF(kFF);
    rightLCM_PID.SetOutputRange(kMinOutput, kMaxOutput);

void LauncherController::setPosition(double requestedAngle){
    // Converts angle in degrees to steps
    double steps = armAngle * 0.0439453125;

    // Set arm motors to desired position using SetReference
    leftLCM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition);
    rightLCM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition);
}