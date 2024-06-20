/*
 * Project: ArmController
 * File: ArmController.cpp
 * Description: Take angle as an input then move and hold arm at that angle.
 */

#include "rev/CANSparkMax.h"
#include "subsystems/ArmController.h"
#include "cmath"
#include <frc/smartdashboard/SmartDashboard.h>

ArmController::ArmController(const int leftACMID, const int rightACMID):
    // Create motors
    leftACM(leftACMID, rev::CANSparkMax::MotorType::kBrushless),
    rightACM(rightACMID, rev::CANSparkMax::MotorType::kBrushless) {

    // Restore factory defaults because all the examples do it
    leftACM.RestoreFactoryDefaults();
    rightACM.RestoreFactoryDefaults();

    // Set motor current limits to prevent them from cooking
    leftACM.SetSmartCurrentLimit(40);
    rightACM.SetSmartCurrentLimit(40);

    // Set modes to brake
    leftACM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightACM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

	// Set PID feedback
    leftACM_PID.SetFeedbackDevice(leftACME);
    rightACM_PID.SetFeedbackDevice(leftACME);

    // Set conversion factor (2pi for absolute, .0725/2 for relative)
    leftACME.SetPositionConversionFactor(2 * M_PI);//(.0725 / 2);
    //rightACME.SetPositionConversionFactor(2 * M_PI);

    //rightACME.SetInverted(true);
	//leftACME.SetInverted(true);

    // Set PID coefficients
    leftACM_PID.SetP(0.6);
    leftACM_PID.SetI(1e-4);
    leftACM_PID.SetD(0);
    leftACM_PID.SetIZone(0);
    leftACM_PID.SetFF(0);
    leftACM_PID.SetOutputRange(-.3, .3);


    rightACM_PID.SetP(0.6);
    rightACM_PID.SetI(1e-4);
    rightACM_PID.SetD(0);
    rightACM_PID.SetIZone(0);
    rightACM_PID.SetFF(0);
    rightACM_PID.SetOutputRange(.3, -.3);

    //rightACM.SetInverted(true);

    leftACM_PID.SetPositionPIDWrappingEnabled(true);
    rightACM_PID.SetPositionPIDWrappingEnabled(true);

    leftACM_PID.SetPositionPIDWrappingMinInput(0);
    leftACM_PID.SetPositionPIDWrappingMaxInput(2 * M_PI);
    rightACM_PID.SetPositionPIDWrappingMinInput(0);
    rightACM_PID.SetPositionPIDWrappingMaxInput(2 * M_PI);

    // Burn flash
    leftACM.BurnFlash();
    rightACM.BurnFlash();
}

// Call this to get the position value of the arm system
double ArmController::GetRArmPosition() {
    double position = leftACME.GetPosition();//rightACME.GetPosition();
    return position;
}
double ArmController::GetLArmPosition() {
    double position = leftACME.GetPosition();
    return position;
}

bool ArmController::SetArmPosition(double angle){
	double threePiTwo = (3 * M_PI) / 2.0;
	double pi = M_PI;
	double piTwo = M_PI / 2.0;
	double twoPi = M_PI * 2.0;
	double currentPosition = leftACME.GetPosition();
	double output;

	output = twoPi - angle;

	/*if (angle <= 0) { // Force it back to 0 position

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
	} */

	// i hope this works
	// i am at the brink of delerium

    // Set arm motors to desired position using SetReference
    leftACM_PID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);
    rightACM_PID.SetReference(output, rev::CANSparkMax::ControlType::kPosition);

    frc::SmartDashboard::PutNumber("LARMPID", output);
    frc::SmartDashboard::PutNumber("RARMPID", output);

	return ((leftACME.GetPosition() >= (output - .25)) && (leftACME.GetPosition() <= (output + .25)));
}