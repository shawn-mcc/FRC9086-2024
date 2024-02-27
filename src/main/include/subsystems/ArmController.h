/*
 * Project: ArmController
 * File: ArmController.cpp
 * Description: Take angle as an input then move and hold arm at that angle.
 */

#pragma once

#include <rev/CANSparkMax.h>

class ArmController {
public:
    ArmController(const int leftACM, const int rightACM);

    void SetPosition(double angle);

private:

    //PID coefficients
    double kP = 0.1,
           kI = 1e-4,
           kD = 1,
           kIz = 0,
           kFF = 0,
           kMaxOutput = 1,
           kMinOutput = -1;

    // Get motor controllers
    rev::CANSparkMax leftACM;
    rev::CANSparkMax rightACM;

    // Create encoders
    rev::SparkAbsoluteEncoder leftACME = leftACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkAbsoluteEncoder rightACME = rightACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Create PID controllers
    rev::SparkPIDController leftACM_PID = leftACM.GetPIDController();
    rev::SparkPIDController rightACM_PID = rightACM.GetPIDController();

};

