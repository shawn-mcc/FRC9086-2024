#pragma once

#include <rev/CANSparkMax.h>

class ArmController {
public:
    ArmController(const int leftACM, const int rightACM);

    void setPosition(double angle);

private:
    rev::CANSparkMax leftACM;
    rev::CANSparkMax rightACM;

    // Create encoders
    rev::SparkAbsoluteEncoder leftACME = leftACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkAbsoluteEncoder rightACME = rightACM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Create PID controllers
    rev::SparkPIDController leftACM_PID = leftACM.GetPIDController();
    rev::SparkPIDController rightACM_PID = rightACM.GetPIDController();

};

