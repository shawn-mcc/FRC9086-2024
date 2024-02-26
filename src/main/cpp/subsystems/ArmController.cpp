/*
 * Project: ArmController
 * File: ArmController.cpp
 * Description: Take angle as an input then move and hold arm at that angle.
 */
#include "rev/CANSparkMax.h"
#include "subsystems/ArmController.h"

//PID coefficients
double kP = 0.1,
       kI = 1e-4, kD = 1,
       kIz = 0, kFF = 0,
       kMaxOutput = 1,

       kMinOutput = -1;

ArmController::ArmController(const int leftACMID, const int rev::CANSparkMax rightACMID)
    // Create motors
    rev::CANSparkMax leftACM{leftACMID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax rightACM{rightACMID, rev::CANSparkMax::MotorType::kBrushless};

    // Restore factory defaults because all the examples do it
    leftACM.RestoreFactoryDefaults();
    rightACM.RestoreFactoryDefaults();

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


void ArmController::setPosition(double angle){
    // Converts angle in degrees to steps
    double steps = angle * 0.0439453125;

    // Set arm motors to desired position using SetReference
    leftACM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition);
    rightACM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition);
}