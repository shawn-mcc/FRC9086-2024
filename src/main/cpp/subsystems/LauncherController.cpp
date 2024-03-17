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

    leftLCM_PID.SetFeedbackDevice(rightLCME);
    rightLCM_PID.SetFeedbackDevice(rightLCME);//(rightLCME);


    // Set PID coefficients
    leftLCM_PID.SetP(0.1);
    leftLCM_PID.SetI(1e-4);
    leftLCM_PID.SetD(1);
    leftLCM_PID.SetIZone(0);
    leftLCM_PID.SetFF(0);
    leftLCM_PID.SetOutputRange(1, -1);


    rightLCM_PID.SetP(0.1);
    rightLCM_PID.SetI(1e-4);
    rightLCM_PID.SetD(1);
    rightLCM_PID.SetIZone(0);
    rightLCM_PID.SetFF(0);
    rightLCM_PID.SetOutputRange(-1, 1);

    leftLCM_PID.SetPositionPIDWrappingEnabled(true);
    rightLCM_PID.SetPositionPIDWrappingEnabled(true);

    leftLCM_PID.SetPositionPIDWrappingMinInput(0);
    leftLCM_PID.SetPositionPIDWrappingMaxInput(2 * M_PI);
    rightLCM_PID.SetPositionPIDWrappingMinInput(0);
    rightLCM_PID.SetPositionPIDWrappingMaxInput(2 * M_PI);

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
    /*double threePiTwo = (3 * M_PI) / 2.0;
    double pi = M_PI;
    double piTwo = M_PI / 2.0;
    double twoPi = M_PI * 2.0;
    double currentPosition = rightLCME.GetPosition();
    double output = 6.27;

    angle = twoPi - angle;

    if (angle <= 0) { // Force it back to 0 position

        output = 6.27;

    } else if ((currentPosition > threePiTwo) && (currentPosition < twoPi)) { // Range 1 (3pi/2 - 2pi)

        if ((angle > threePiTwo) && (angle < twoPi)) { // Check if angle is within range
            output = angle;
        } else if ((angle < threePiTwo) && (angle > 0)){
            output = threePiTwo;
        } else {
            output = 6.27;
        }

    } else if ((currentPosition > pi) && (currentPosition < threePiTwo)) { // Range 1 (pi - 3pi/2)

        if ((angle > pi) && (angle < threePiTwo)) { // Check if angle is within range
            output = angle;
        } else if (angle >= threePiTwo){
            output = threePiTwo;
        } else {
            output = pi;
        }

    } else if ((currentPosition > piTwo) && (currentPosition < pi)) { // Range 1 (pi/2 - pi)

        if ((angle > piTwo) && (angle < pi)) { // Check if angle is within range
            output = angle;
        } else if (angle >= pi){
            output = pi;
        } else {
            output = piTwo;
        }

    } else if ((currentPosition > 0) && (currentPosition < piTwo)) { // Range 1 (0 - pi/2)

        if (angle < piTwo) { // Check if angle is within range
            output = angle;
        } else {
            output = piTwo;
        }

    } else {
        // pid at 1:51am, please healp
    }


    // i hope this works
    // i am at the brink of delerium

    // Set arm motors to desired position using SetReference
    //output = twoPi - angle
    //leftLCM_PID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);
    //rightLCM_PID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);


    //frc::SmartDashboard::PutNumber("LLAUNCHERPID", output);
    //frc::SmartDashboard::PutNumber("RLAUNCHERPID", output); */
}