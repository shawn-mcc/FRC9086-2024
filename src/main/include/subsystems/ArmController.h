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

    double GetRArmPosition();
    double GetLArmPosition();

    bool SetArmPosition(double angle);

private:
	int autonState;

    // Get motor controllers
    rev::CANSparkMax leftACM;
    rev::CANSparkMax rightACM;

    // Create encoders
    //rev::SparkRelativeEncoder leftACME = leftACM.GetEncoder();
    //rev::SparkRelativeEncoder rightACME = rightACM.GetEncoder();
    rev::SparkAbsoluteEncoder leftACME = leftACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkAbsoluteEncoder rightACME = rightACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Create PID controllers
    rev::SparkPIDController leftACM_PID = leftACM.GetPIDController();
    rev::SparkPIDController rightACM_PID = rightACM.GetPIDController();

};

