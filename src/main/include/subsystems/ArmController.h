#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <rev/CANSparkMax.h>

class ArmController {
public:
    ArmController(rev::CANSparkMax& leftACM, rev::CANSparkMax& rightACM);
    void setPosition(double angle);

private:
    rev::CANSparkMax leftACM;
    rev::CANSparkMax rightACM;

    // Create encoders
    rev::SparkMaxAlternateEncoder leftACME = leftACM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192);
    rev::SparkMaxAlternateEncoder rightACME = rightACM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192);

    // Create PID controllers
    rev::SparkPIDController leftACM_PID = leftACM.getPIDController();
    rev::SparkPIDController rightACM_PID = rightACM.getPIDController();

};

#endif //ARMCONTROLLER_H
