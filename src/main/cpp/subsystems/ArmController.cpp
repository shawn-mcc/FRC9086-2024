/*
 * Project: ArmController
 * File: ArmController.cpp
 * Description: Take angle as an input then move and hold arm at that angle.
 */

#include "rev/CANSparkMax.h"
#include "subsystems/ArmController.h"
#include "cmath"

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
    leftACM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightACM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Set conversion factor (2pi for absolute, .0725/2 for relative)
    leftACME.SetPositionConversionFactor(2 * M_PI);//(.0725 / 2);
    //rightACME.SetPositionConversionFactor(2 * M_PI);//(.0725 / 2);

    leftACM_PID.SetFeedbackDevice(leftACME);
    rightACM_PID.SetFeedbackDevice(leftACME);//(rightACME);

    // Set PID coefficients
    leftACM_PID.SetP(0.1);
    leftACM_PID.SetI(1e-4);
    leftACM_PID.SetD(1);
    leftACM_PID.SetIZone(0);
    leftACM_PID.SetFF(0);
    leftACM_PID.SetOutputRange(-1, 1);

    rightACM_PID.SetP(0.1);
    rightACM_PID.SetI(1e-4);
    rightACM_PID.SetD(1);
    rightACM_PID.SetIZone(0);
    rightACM_PID.SetFF(0);
    rightACM_PID.SetOutputRange(-1, 1);

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

void ArmController::SetArmPosition(double angle){

    // Set arm motors to desired position using SetReference
    leftACM_PID.SetReference(-angle, rev::CANSparkMax::ControlType::kPosition);
    rightACM_PID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
}