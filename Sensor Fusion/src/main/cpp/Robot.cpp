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

  timer->Reset();

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
  // supposed to be 1 / 0.51 --> 1.96
  lEncoder.SetPositionConversionFactor(1.96);
  rEncoder.SetPositionConversionFactor(1.96);

  timer->Start();

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
    double gyroVal = imu->GetAngle().value();
    frc::SmartDashboard::PutNumber("gyro value", gyroVal);
    frc::SmartDashboard::PutNumber("adjusted gyro value", gyroVal + gyroDrift);

    units::magnetic_field_strength::tesla_t mag_x_native = imu->GetMagneticFieldX();
    units::magnetic_field_strength::tesla_t mag_y_native = imu->GetMagneticFieldY();

    double mag_x = mag_x_native.value();
    double mag_y = mag_y_native.value();

    double mag_heading = ((atan2(mag_y, mag_x) * 180) / PI);


    // formula to verify gyro and mag: ((gyroVal % 360) + og_mag_heading) % 360 = mag_heading % 360
    static double og_mag_heading = mag_heading;
    frc::SmartDashboard::PutNumber("og mag heading", og_mag_heading);


    // Code below to calculate mag heading noise (TODO: Test!)
    static double og_val = mag_heading;
    static double max_noise = 0;

    if (fabs(mag_heading - og_val) > max_noise) max_noise = fabs(mag_heading - og_val);

    // if ((mag_heading - og_val) > max) max = mag_heading - og_val;
    // if ((og_val - mag_heading) < min) min = og_val - mag_heading;
    // frc::SmartDashboard::PutNumber("min mag heading", min);
    // frc::SmartDashboard::PutNumber("max mag heading", max);

    frc::SmartDashboard::PutNumber("mag heading", mag_heading);
    frc::SmartDashboard::PutNumber("max noise", max_noise);
    frc::SmartDashboard::PutNumber("noise percentage", (max_noise / (og_val + max_noise)) * 100);



    // TODO: drift sensor to each other
    // 0.01 * error * time interval
    // Kalman sensor - probability state filter

    // theta_m (mag_heading) -->   N% (noise percentage) =  +/- 1.2 / (69.7+1.2) = 0.017 --> 1.7 % 
    // theat_g  (gyroVal) -->  [d/s] (drift per second) = 

    // adjusts raw gyro and magnetometer (TODO: Test!)
    double gyro_adjusted_val = fmod(fmod(gyroVal, 360) + og_mag_heading, 360);
    double mag_adjusted_val = fmod(mag_heading, 360);

    // observed  noise of magnetometer and drift of gyro
    double noise_p = 0.017; 
    double max_drift_per_sec = 0; //find 

    double error_delta = gyroVal - mag_heading;
    double error_past_bounds = fabs(error_delta) - (mag_heading + (mag_heading * noise_p) / 2);

    double currentTime = timer->GetFPGATimestamp().value();
    double elapsedTime = currentTime - lastTime;

    // error is greater than possible noise of magnetometer 
    if (fabs(error_delta) > noise_p * mag_heading) {

      // drifts gyro value to magnetometer heading at max drift per sec (TODO: Test!)
      if (elapsedTime > 1) {
        gyroDrift -= max_drift_per_sec;
        lastTime = currentTime;
      }
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
