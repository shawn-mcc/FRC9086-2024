#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "rev/CANSparkMax.h"
#include "cameraserver/CameraServer.h"
#include <wpi/raw_ostream.h>
#include "frc/XboxController.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <stdlib.h>



class Robot : public frc::TimedRobot {
  //Setup motors
  rev::CANSparkMax m_leftFront{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftBack{3, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFront{4, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightBack{5, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_scoop{7, rev::CANSparkMax::MotorType::kBrushless};

  //rev::SparkMaxRelativeEncoder m_scoopEncoder{rev::CANSparkMax::MotorType::kBrushless};

  // rev::SparkMaxRelativeEncoder encoder = m_horizontalArm.GetEncoder();

  // //Setup vertical PID stuff for vertical
  // rev::SparkMaxPIDController m_pidController = m_motor.GetPIDController();
  // rev::SparkMaxRelativeEncoder m_encoder = m_motor.GetEncoder();

  // // default PID coefficients
  // double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // // default smart motion coefficients
  // double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;


  // // motor max RPM
  // const double MaxRPM = 5700;


  frc::DifferentialDrive robotDrive{m_leftFront, m_rightFront};
  frc::Joystick m_driverController{0};
  //frc::XboxController m_driverARMController{1};
  cs::UsbCamera camera1;

 public:
  void RobotInit() override { //Everything here is ran one time before robot intitializes.
    m_leftBack.Follow(m_leftFront);
    m_rightBack.Follow(m_rightFront);


    //camera1 = frc::CameraServer::StartAutomaticCapture();
    //camera1.SetResolution(320, 240);
    //camera1.SetFPS(15);


    //m_scoopEncoder.SetPosition(0);

    //     /**
    //  * The RestoreFactoryDefaults method can be used to reset the configuration parameters
    //  * in the SPARK MAX to their factory default state. If no argument is passed, these
    //  * parameters will not persist between power cycles
    //  */
    // m_motor.RestoreFactoryDefaults();

    // // set PID coefficients
    // m_pidController.SetP(kP);
    // m_pidController.SetI(kI);
    // m_pidController.SetD(kD);
    // m_pidController.SetIZone(kIz);
    // m_pidController.SetFF(kFF);
    // m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // /**
    //  * Smart Motion coefficients are set on a SparkMaxPIDController object
    //  *
    //  * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
    //  * the pid controller in Smart Motion mode
    //  * - SetSmartMotionMinOutputVelocity() will put a lower bound in
    //  * RPM of the pid controller in Smart Motion mode
    //  * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
    //  * of the pid controller in Smart Motion mode
    //  * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
    //  * error for the pid controller in Smart Motion mode
    //  */
    // m_pidController.SetSmartMotionMaxVelocity(kMaxVel);
    // m_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
    // m_pidController.SetSmartMotionMaxAccel(kMaxAcc);
    // m_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

    // // display PID coefficients on SmartDashboard
    // frc::SmartDashboard::PutNumber("P Gain", kP);
    // frc::SmartDashboard::PutNumber("I Gain", kI);
    // frc::SmartDashboard::PutNumber("D Gain", kD);
    // frc::SmartDashboard::PutNumber("I Zone", kIz);
    // frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    // frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    // frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    // // display Smart Motion coefficients
    // frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    // frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
    // frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    // frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    // frc::SmartDashboard::PutNumber("Set Position", 0);
    // frc::SmartDashboard::PutNumber("Set Velocity", 0);

    // // button to toggle between velocity and smart motion modes
    // frc::SmartDashboard::PutBoolean("Mode", false);

  }
  void TeleopInit() override{
    //Initialize all parameters

    //Set motors to brake mode to begin and allow users to set to coast
    frc::SmartDashboard::PutString("Brake Mode", "Brake");
    m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  }
  void TeleopPeriodic() override { //Teleoperated mode (wireless)

    //auto rotations = m_scoopEncoder.GetPosition();

    float joystickOffsetX = 0.0; /*Offset added to joystick input to eliminate deadzones,
                                if you notice deadzones in any joystick increase this value */
    float joystickOffsetY = 0.0;

    float throttleControl = 1 - m_driverController.GetRawAxis(3);
    float turnControl = 1;

    //float rotation = 1/*abs(m_driverController.GetRawAxis(2))*/; //Allow robot to spin only when rotation is applied to the joystick

    float xAxis = m_driverController.GetX() * turnControl + joystickOffsetX; //Get X axis on joystick and add joystick_Offset
    float yAxis = m_driverController.GetY() * throttleControl + joystickOffsetY; //Get Y axis on joystick and add joystick_Offset

    //Establish deadzones at 3% in each direction
    if ((xAxis >= -.03) && (xAxis <= .03) && (yAxis >= -.03) && (yAxis <= .03)){
      m_leftFront.Set(0);
      m_rightFront.Set(0);
    }
    else{
      m_leftFront.Set(-xAxis + yAxis);
      m_rightFront.Set(-xAxis - yAxis);
    }

    //Change idle modes from coast to brake and brake to coast
    bool buttonStat = m_driverController.GetRawButtonPressed(2);

    if (buttonStat == true){
      if (m_leftFront.GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake){
        frc::SmartDashboard::PutString("Brake Mode", "Coast");
        m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      }
      else if (m_leftFront.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast){
        frc::SmartDashboard::PutString("Brake Mode", "Brake");
        m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      }
      sleep(.01);
    }



    // float verticalControl = m_driverARMController.GetRawAxis(1);
    // double SetPoint = verticalControl * 100;
    //     // read PID coefficients from SmartDashboard
    // double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    // double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    // double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    // double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    // double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    // double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    // double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    // double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    // double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
    // double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    // double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP))   { m_pidController.SetP(p); kP = p; }
    // if((i != kI))   { m_pidController.SetI(i); kI = i; }
    // if((d != kD))   { m_pidController.SetD(d); kD = d; }
    // if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { m_pidController.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    // if((maxV != kMaxVel)) { m_pidController.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    // if((minV != kMinVel)) { m_pidController.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    // if((maxA != kMaxAcc)) { m_pidController.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    // if((allE != kAllErr)) { m_pidController.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }

    // double ProcessVariable;
    // bool mode = frc::SmartDashboard::GetBoolean("Mode", true);
    // //SetPoint = verticalControl;
    // m_pidController.SetReference(SetPoint, rev::CANSparkMax::ControlType::kVelocity);
    // ProcessVariable = m_encoder.GetVelocity();
    // if(mode) {
    //   SetPoint = verticalControl;
    //   m_pidController.SetReference(SetPoint, rev::CANSparkMax::ControlType::kVelocity);
    //   ProcessVariable = m_encoder.GetVelocity();
    // } else {
    //   SetPoint = frc::SmartDashboard::GetNumber("Set Position", 0);
    //   /**
    //    * As with other PID modes, Smar  8t Motion is set by calling the
    //    * SetReference method on an existing pid object and setting
    //    * the control type to kSmartMotion
    //    */
    //   m_pidController.SetReference(SetPoint, rev::CANSparkMax::ControlType::kSmartMotion);
    //   ProcessVariable = m_encoder.GetPosition();
    // }

    // frc::SmartDashboard::PutNumber("Set Point", SetPoint);
    // frc::SmartDashboard::PutNumber("Process Variable", ProcessVariable);
    // frc::SmartDashboard::PutNumber("Output", m_motor.GetAppliedOutput());


  }

  void TestInit() override {
  }
  void TestPeriodic() override {
    frc::SmartDashboard::PutNumber("Timing Cycles", 0);
    m_scoop.Set(0.5);
    // auto rotations = m_scoopEncoder.GetPosition();
    // frc::SmartDashboard::PutNumber("Rotations", rotations);

    // if(rotations == 90);
    //   m_scoop.Set(0);
    // if(rotations < 90);
    //   m_scroop.Set(0.25);
    // if(rotations > 90);
    //   m_scoop.Set(-0.25);

  }

  void AutonomousInit() override{
      frc::SmartDashboard::PutNumber("Timing Cycles", 0);
  }

  void AutonomousPeriodic() override{
    //set constant data points
    float tuneLeft = -1;
    float tuneRight = .9;

    //basic speed data below:
    //20ft/s at 100% power due to 5ft/s at 25% power
    //to edge of charge pad is 10ft 1/8in.
    //6ft 1/4in. across charge pad
    //18ft total safe spread
    //20ft max spread with no tolerance

    //begin drive code
    double timeConsumed = frc::SmartDashboard::GetNumber("Timing Cycles",0);
    if (timeConsumed <= 175){ //Go over and past the charge pad
      m_leftFront.Set(-.50 * tuneLeft);
      m_rightFront.Set(-.50 * tuneRight);
    }
    else if ((timeConsumed > 175) && (timeConsumed <= 225)){ //Reverse onto charge pad
      m_leftFront.Set(0 * tuneLeft);
      m_rightFront.Set(0.05 * tuneRight);
    }
    else if ((timeConsumed > 225) && (timeConsumed <= 265)){ //Reverse onto charge pad
      m_leftFront.Set(.5 * tuneLeft);
      m_rightFront.Set(.5 * tuneRight);
    }
    else if ((timeConsumed > 265) && (timeConsumed <= 1050)){ //stop on charge pad
      m_leftFront.Set(0 * tuneLeft);
      m_rightFront.Set(0 * tuneRight);
    }
    else if ((timeConsumed > 1050) && (timeConsumed <= 1300)){ //reverse into comm. zone
      m_leftFront.Set(.25 * tuneLeft);
      m_rightFront.Set(.25 * tuneRight);
    }
    else if (timeConsumed > 1300){ //stop *near* the start
      m_leftFront.Set(0);
      m_rightFront.Set(0);
    }
    sleep(.01);
    frc::SmartDashboard::PutNumber("Timing Cycles", timeConsumed + 1);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif