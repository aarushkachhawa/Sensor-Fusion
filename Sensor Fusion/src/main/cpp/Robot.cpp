// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  lMotor->RestoreFactoryDefaults();
  lMotorFollower->RestoreFactoryDefaults();
  rMotor->RestoreFactoryDefaults();
  rMotorFollower->RestoreFactoryDefaults();

  lMotor->SetInverted(true);
  lMotorFollower->Follow(*lMotor, false);
  rMotor->SetInverted(false);
  rMotorFollower->Follow(*rMotor, false);

  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);
  
  imu->Reset();

}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y", stick->GetRawAxis(1));
  frc::SmartDashboard::PutNumber("right x", stick->GetRawAxis(4));
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);

}
void Robot::TeleopPeriodic() {
  // L_X = 0, L_Y = 1, R_X = 4, R_Y = 5
  // Buttons: A = , B = , X = , Y = 

  left_y = stick->GetRawAxis(1);
  right_x = stick->GetRawAxis(4);

  robotDrive->ArcadeDrive(left_y, right_x);

  if (stick->GetRawButtonPressed(1)) {
    // theta represents the change in orientation (in radians) from the original position

    double lRotations = lEncoder.GetPosition(); 
    double rRotations = rEncoder.GetPosition();
    // TODO: Convert native encoder units to centimeters
    
    double theta = (lRotations - rRotations) / (rDistanceToCenter + lDistanceToCenter);
    frc::SmartDashboard::PutNumber("theta", theta);
    float gyroVal = imu->GetAngle().value();
    frc::SmartDashboard::PutNumber("gyro value", gyroVal);

  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
