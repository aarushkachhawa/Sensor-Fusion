// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Joystick.h>
#include "SFDrive.h"
#include "SensorDriftModule.h"

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

  rev::SparkMaxRelativeEncoder lEncoder_og = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder_og = rMotor->GetEncoder();

  rev::SparkMaxRelativeEncoder* lEncoder = &lEncoder_og;
  rev::SparkMaxRelativeEncoder* rEncoder = &rEncoder_og;

  // std::unique_ptr<rev::SparkMaxRelativeEncoder> lEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(lEncoder_og);
  // std::unique_ptr<rev::SparkMaxRelativeEncoder> rEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(rEncoder_og);
  
  

  Joystick* stick = new Joystick(0);

  SFDrive* robotDrive = new SFDrive(lMotor, rMotor);
  SensorDriftModule* sensorDrift = new SensorDriftModule(lEncoder, rEncoder);



};
