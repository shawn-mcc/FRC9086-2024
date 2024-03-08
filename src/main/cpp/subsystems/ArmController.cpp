/*
 * Project: ArmController
 * File: ArmController.cpp
 * Description: Take angle as an input then move and hold arm at that angle.
 */

#include "rev/CANSparkMax.h"
#include "subsystems/ArmController.h"

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

    // Set PID coefficients
    leftACM_PID.SetP(kP);
    leftACM_PID.SetI(kI);
    leftACM_PID.SetD(kD);
    leftACM_PID.SetIZone(kIz);
    leftACM_PID.SetFF(kFF);
    leftACM_PID.SetOutputRange(kMinOutput, kMaxOutput);

    rightACM_PID.SetP(kP);
    rightACM_PID.SetI(kI);
    rightACM_PID.SetD(kD);
    rightACM_PID.SetIZone(kIz);
    rightACM_PID.SetFF(kFF);
    rightACM_PID.SetOutputRange(kMinOutput, kMaxOutput);

    rightACM.Follow(leftACM);

    // Burn flash
    leftACM.BurnFlash();
    rightACM.BurnFlash();
}

void ArmController::SetPosition(double angle){

    // Set arm motors to desired position using SetReference
    leftACM_PID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
    rightACM_PID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
}