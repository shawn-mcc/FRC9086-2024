/*
* Project: FireControl
 * File: FireControl.cpp
 * Description: Engages the firing mechanisms
 */

#pragma once

#include <rev/CANSparkMax.h>

class FireControl {
public:
    FireControl(const int topMotor, const int bottomMotor, const int pusherMotor);

    // Spool up the outer wheels
    void Spool(double speed);

    // Activate inner wheels (forces outer on)
    void Fire(double speed);

    // All wheels moving reverse
    void Intake();

    // Stop all wheels
    void StopAll();

private:

    // Get motor controllers
    rev::CANSparkMax topM;
    rev::CANSparkMax bottomM;
    rev::CANSparkMax pusherM;

};

