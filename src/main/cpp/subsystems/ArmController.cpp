#include <rev/CANSparkMax.h>
#include <cmath>

//PID coefficients
double kP = 0.1,
       kI = 1e-4, kD = 1,
       kIz = 0, kFF = 0,
       kMaxOutput = 1,
       kMinOutput = -1;

//Arm & Launcher lengths in inches
double armLen = 24,
       launcherLen = 12;

ArmController::ArmController(rev::CANSparkMax leftACM, rev::CANSparkMax rightACM, int angle):

       // Restore factory defaults because all the examples do it
       leftACM.RestoreFactoryDefaults();
       rightACM.RestoreFactoryDefaults();

       // Create encoders
       rev::SparkMaxAlternateEncoder leftACME = leftACM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192)
       rev::SparkMaxAlternateEncoder rightACME = rightACM.GetAlternateEncoder(rev::CANSparkMax::MotorType::kBrushless, 8192)

       // Create PID controllers
       rev::SparkPIDController leftACM_PID = leftACM.getPIDController();
       rev::SparkPIDController rightACM_PID = rightACM.getPIDController();

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

void ArmController::setPosition(double requestedAngle, double launcherAngle){
       // Funny trig to determine if desired position will result in exceeding horizontal and vertical constraints
       double trueLauncherAngle = launcherAngle + armAngle

       double armRadians = armAngle * M_PI / 180;
       double launcherRadians = trueLauncherAngle * M_PI / 180;

       double y1 = armLen * sin(armRadians)
       double y2 = launcherLen * sin(launcherAngle)

       double TEL = y1 + y2

       if (TEL > maximumExtensionLength){
              // figure out how not to be
       } else {
              // Converts angle in degrees to steps
              double steps = armAngle * 0.0439453125

              // Set arm motors to desired position using SetReference
              leftACM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition)
              rightACM_PID.SetReference(steps, rev::CANSparkMax::ControlType::kPosition)
       }
}