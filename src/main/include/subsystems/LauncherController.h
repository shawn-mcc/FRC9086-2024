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

    double GetLauncherPosition();

    // Set position of the launcher controller
    void SetLauncherPosition(double requestedAngle);

private:
    // Get motor controllers
    rev::CANSparkMax leftLCM;
    rev::CANSparkMax rightLCM;

    // Create encoders
    //rev::SparkAbsoluteEncoder leftLCME = leftLCM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkAbsoluteEncoder rightLCME = rightLCM.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    // Create PID controllers
    rev::SparkPIDController leftLCM_PID = leftLCM.GetPIDController();
    rev::SparkPIDController rightLCM_PID = rightLCM.GetPIDController();

};