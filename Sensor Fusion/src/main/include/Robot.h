// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Joystick.h>
#include "SFDrive.h"
#include <frc/ADIS16448_IMU.h>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/Timer.h>

// change ids
#define lMotorLeaderID 15
#define lMotorFollowerID 16
#define rMotorLeaderID 9
#define rMotorFollowerID 3

#define PI 3.141592

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  double left_y = 0.0;
  double right_x = 0.0;

  rev::CANSparkMax* lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  rev::SparkMaxRelativeEncoder lEncoder = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = rMotor->GetEncoder();

  Joystick* stick = new Joystick(0);

  // in inches
  const double rDistanceToCenter = 13.0; 
  const double lDistanceToCenter = 13.0;

  // gyro
  frc::ADIS16448_IMU* imu = new ADIS16448_IMU();

  // ^ add magnetometer and report pressure
  // order proximity sensor


  SFDrive* robotDrive = new SFDrive(lMotor, rMotor);

  Timer* timer = new Timer(); 
  double lastTime = 0;

  double gyroDrift = 0;

};
