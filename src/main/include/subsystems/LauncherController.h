/*
 * Project: Launcher Controller
 * File: LauncherController.h
 * Description: Control launcher position
 */

#pragma once

#include <rev/CANSparkMax.h>

class LauncherController {
public:
    // Generate a launcher controller object
    LauncherController(const int leftLCMID, const int rightLCMID);

    // Set position of the launcher controller
    void SetPosition(double requestedAngle);

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
    rev::CANSparkMax leftLCM;
    rev::CANSparkMax rightLCM;

    // Create encoders
    rev::SparkAbsoluteEncoder leftLCME = leftLCM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkAbsoluteEncoder rightLCME = rightLCM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Create PID controllers
    rev::SparkPIDController leftLCM_PID = leftLCM.GetPIDController();
    rev::SparkPIDController rightLCM_PID = rightLCM.GetPIDController();

};