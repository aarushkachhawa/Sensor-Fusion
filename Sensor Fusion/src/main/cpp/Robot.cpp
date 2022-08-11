// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

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

  lMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  lMotorFollower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rMotorFollower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);


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

  // orignallly 0.51 (6.4 / (4*pi)) where 6.4 is gear ration and 4pi is circumference
  // supposed to be 1 / 0.51
  lEncoder.SetPositionConversionFactor(1.96);
  rEncoder.SetPositionConversionFactor(1.96);

}
void Robot::TeleopPeriodic() {
  // L_X = 0, L_Y = 1, R_X = 4, R_Y = 5
  // Buttons: A = 1, B = , X = , Y = 

  left_y = stick->GetRawAxis(1);
  right_x = stick->GetRawAxis(4);

  robotDrive->ArcadeDrive(left_y, (-1) * right_x);

  // theta represents the change in orientation (in radians) from the original position

    double lRotations = lEncoder.GetPosition(); 
    double rRotations = rEncoder.GetPosition();
    
    double theta = (lRotations - rRotations) / (rDistanceToCenter + lDistanceToCenter);
    theta = theta * 180 / PI;
    frc::SmartDashboard::PutNumber("computed theta", theta);
    float gyroVal = imu->GetAngle().value();
    frc::SmartDashboard::PutNumber("gyro value", gyroVal);

    // TODO: magnetometer heading
    units::magnetic_field_strength::tesla_t mag_x_native = imu->GetMagneticFieldX();
    units::magnetic_field_strength::tesla_t mag_y_native = imu->GetMagneticFieldY();

    double mag_x = mag_x_native.value();
    double mag_y = mag_y_native.value();

    double mag_heading = ((atan2(mag_y, mag_x) * 180) / PI);
    frc::SmartDashboard::PutNumber("mag heading", mag_heading);

    // TODO: drift sensor to each other
    // 0.01 * error * time interval
    // Kalman sensor - probability state filter
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
