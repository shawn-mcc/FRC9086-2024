/*
* Project: LauncherController
 * File: LauncherController.cpp
 * Description: Take angle as an input then move and hold launcher at that angle.
 */

#include "subsystems/LauncherController.h"
#include <rev/CANSparkMax.h>
#include "cmath"

LauncherController::LauncherController(const int leftLCMID, const int rightLCMID):
    // Create motors
    leftLCM(leftLCMID, rev::CANSparkMax::MotorType::kBrushless),
    rightLCM(rightLCMID, rev::CANSparkMax::MotorType::kBrushless) {

    // Restore factory defaults because all the examples do it
    leftLCM.RestoreFactoryDefaults();
    rightLCM.RestoreFactoryDefaults();

    // Set motor current limits to prevent them from cooking
    leftLCM.SetSmartCurrentLimit(40);
    rightLCM.SetSmartCurrentLimit(40);

    // Set modes to brake
    leftLCM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightLCM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Set conversion factor (2pi for absolute, .0725/2 for relative)
    //leftLCME.SetPositionConversionFactor(2 * M_PI);
    rightLCME.SetPositionConversionFactor(2 * M_PI);

	//rightLCME.SetInverted(true);

    //leftLCM_PID.SetFeedbackDevice(rightLCME);
    rightLCM_PID.SetFeedbackDevice(rightLCME);//(rightLCME);

/*
    // Set PID coefficients
    leftLCM_PID.SetP(kP);
    leftLCM_PID.SetI(kI);
    leftLCM_PID.SetD(kD);
    leftLCM_PID.SetIZone(kIz);
    leftLCM_PID.SetFF(kFF);
    leftLCM_PID.SetOutputRange(kMinOutput, kMaxOutput);
	*/

    rightLCM_PID.SetP(kP);
    rightLCM_PID.SetI(kI);
    rightLCM_PID.SetD(kD);
    rightLCM_PID.SetIZone(kIz);
    rightLCM_PID.SetFF(kFF);
    rightLCM_PID.SetOutputRange(kMinOutput, kMaxOutput);

    // Burn flash
    leftLCM.BurnFlash();
    rightLCM.BurnFlash();
}

// Call this to get the position value of the launcher system
double LauncherController::GetLauncherPosition() {
    double position = rightLCME.GetPosition();
    return position;
}

void LauncherController::SetLauncherPosition(double requestedAngle){

    // Set arm motors to desired position using SetReference
    //leftLCM_PID.SetReference(requestedAngle, rev::CANSparkMax::ControlType::kPosition);
    rightLCM_PID.SetReference((2 * M_PI) - requestedAngle, rev::CANSparkMax::ControlType::kPosition);
}