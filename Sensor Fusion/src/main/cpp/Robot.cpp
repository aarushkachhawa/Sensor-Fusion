// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  lMotor->RestoreFactoryDefaults();
  lMotorFollower->RestoreFactoryDefaults();
  rMotor->RestoreFactoryDefaults();
  rMotorFollower->RestoreFactoryDefaults();

  lMotor->SetInverted(true);
  lMotorFollower->Follow(*lMotor, false);
  rMotor->SetInverted(false);
  rMotorFollower->Follow(*rMotor, false);

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // L_X = 0, L_Y = 1, R_X = 4, R_Y = 5


  double dz = 0.08; // Deadzone
  double x = y_j->GetY(); // Raw Y value

  if (abs(x) > dz) {

    double y = (1/(1-dz)) * abs(x) + (1 - (1/(1-dz)));
    y = copysign(y, x);

    //y = y * y; increased sensitivity

    lMotor->Set(y);
    rMotor->Set(-y);

  } else {
    lMotor->Set(0);
    rMotor->Set(0);
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
