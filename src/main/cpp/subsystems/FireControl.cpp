/*
* Project: FireControl
 * File: FireControl.cpp
 * Description: Engages the firing mechanisms
 */

#include "subsystems/FireControl.h"

FireControl::FireControl(const int topMotor, const int bottomMotor, const int pusherMotor):
    // Create motors
    topM(topMotor, rev::CANSparkMax::MotorType::kBrushless),
    bottomM(bottomMotor, rev::CANSparkMax::MotorType::kBrushless),
    pusherM(pusherMotor, rev::CANSparkMax::MotorType::kBrushless) {

    // Reset to the defaults to be safe
    topM.RestoreFactoryDefaults();
    bottomM.RestoreFactoryDefaults();
    pusherM.RestoreFactoryDefaults();

    // Set motor current limits to prevent them from cooking
    topM.SetSmartCurrentLimit(40);
    bottomM.SetSmartCurrentLimit(40);
    pusherM.SetSmartCurrentLimit(40);

    // Set modes to brake
    topM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    bottomM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    pusherM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // Burn flash to save to memory
    topM.BurnFlash();
    bottomM.BurnFlash();
    pusherM.BurnFlash();
}

// Spool up the outer wheels
void FireControl::Spool(double speed, double trajectory) {
    if (trajectory > 0) {
        topM.Set(speed - trajectory);
        bottomM.Set(speed);
    } else if (trajectory < 0) {
        topM.Set(speed);
        bottomM.Set(speed - fabs(trajectory));
    } else {
        topM.Set(speed);
        bottomM.Set(speed);
    }
}

// Activate inner wheels (forces outer on)
void FireControl::Fire(double speed, double trajectory) {
    Spool(speed, trajectory);
    pusherM.Set(speed);
}

// All wheels moving reverse
void FireControl::Intake() {
    topM.Set(-1);
    bottomM.Set(-1);
    pusherM.Set(-1);
}

// Stop all wheels
void FireControl::StopAll() {
    topM.Set(0);
    bottomM.Set(0);
    pusherM.Set(0);
}