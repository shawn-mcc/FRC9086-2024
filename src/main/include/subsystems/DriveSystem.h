#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"

class DriveSystem : public frc2::SubsystemBase {
 public:
  DriveSystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  void TeleopDrive(double x1Input, double y1Input, double x2Input, double y2Input);

  void AutonDrive();

  void TestDrive();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule swrv_frontLeft;
  SwerveModule swrv_frontRight;
  SwerveModule swrv_backLeft;
  SwerveModule swrv_backRight;


};
