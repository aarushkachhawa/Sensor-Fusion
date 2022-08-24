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
    frc::SmartDashboard::PutNumber("drifted gyro value", gyroVal + gyroDrift);

    units::magnetic_field_strength::tesla_t mag_x_native = imu->GetMagneticFieldX();
    units::magnetic_field_strength::tesla_t mag_y_native = imu->GetMagneticFieldY();

    double mag_x = mag_x_native.value();
    double mag_y = mag_y_native.value();

    double mag_heading = ((atan2(mag_y, mag_x) * 180) / PI);


    static double og_mag_heading = mag_heading;
    frc::SmartDashboard::PutNumber("og mag heading", og_mag_heading);
    frc::SmartDashboard::PutNumber("mag heading", mag_heading);


    // Code below to calculate mag heading noise (TODO: Test!)
    // static double og_val = mag_heading;
    // static double max_noise = 0;

    // if (fabs(mag_heading - og_val) > max_noise) max_noise = fabs(mag_heading - og_val);
    
    // frc::SmartDashboard::PutNumber("max noise", max_noise);
    // frc::SmartDashboard::PutNumber("noise percentage", (max_noise / (og_val + max_noise)) * 100);


    // Code below to calculate max drift from gyro

    // static double maxGyroDriftPerSec = 0;
    // static double previousDrift = gyroVal;
    // double driftCurrentTime = timer->GetFPGATimestamp().value();
    // double driftElapsedTime = driftCurrentTime - driftLastTime;

    // if (driftElapsedTime > 1) {
    //   if (fabs(previousDrift - gyroVal) > maxGyroDriftPerSec){
    //     maxGyroDriftPerSec = fabs(previousDrift - gyroVal);
    //   }
    //   previousDrift = gyroVal;
    //   driftLastTime = driftCurrentTime;
    // }

    // frc::SmartDashboard::PutNumber("maxGryoDriftPerSec", maxGyroDriftPerSec);




    // TODO: drift sensor to each other

    // theta_m (mag_heading) -->   N% (noise percentage) =  +/- 1.2 / (69.7+1.2) = 0.017 --> 1.7 % 
    // theat_g  (gyroVal) -->  [d/s] (drift per second) = 0.4


    // double gyro_adjusted_val = fmod_s(fmod_s(gyroVal, 360) + og_mag_heading, 360); 
    // double mag_adjusted_val = fmod_s(mag_heading, 360);

    double mag_adjusted_val = mag_heading;
    //double gyro_adjusted_val = fmod(gyroVal, 360) + (gyroVal < 0 ? 360 : 0); // (-360, 360) -> 
    double gyro_adjusted_val = fmod((-gyroVal + gyroDrift + og_mag_heading), 360); // gyro val is negative because imu is backwards
    if (gyro_adjusted_val < -180) gyro_adjusted_val += 360;
    else if (gyro_adjusted_val > 180) gyro_adjusted_val -= 360; 

    frc::SmartDashboard::PutNumber("gyro adjusted val", gyro_adjusted_val);
    frc::SmartDashboard::PutNumber("mag adjusted val", mag_adjusted_val);
    frc::SmartDashboard::PutNumber("error", mag_adjusted_val - gyro_adjusted_val);
    frc::SmartDashboard::PutNumber("drift", gyroDrift);

    // observed  noise of magnetometer and drift of gyro (tune further)
    double const noise_p = 0.02; // 0.017 
    double const max_drift_per_sec = 0.5; // 0.4

    double error_delta = mag_adjusted_val - gyro_adjusted_val;
    //double error_past_bounds = fabs(error_delta) - (mag_adjusted_val + (mag_adjusted_val * noise_p) / 2);
    double error_bound = noise_p * mag_adjusted_val;

    double currentTime = timer->GetFPGATimestamp().value();
    double elapsedTime = currentTime - lastTime;

    // TODO: Need logic addressing the wrap around

    if (elapsedTime > 1) {

      // drifts gyro value to magnetometer heading at max drift per sec (TODO: Test!)
      if (fabs(error_delta) > error_bound) {

        // use copysign instead
        if (error_delta > 0) gyroDrift += max_drift_per_sec;
        else gyroDrift -= max_drift_per_sec;
      }    

      // drifts gyro value to magnetometer heading relative to how far in the bounds the gyro value is
      else {
        double drift_percentage = (error_bound - error_delta) / error_bound;

        if (error_delta > 0) gyroDrift += drift_percentage * max_drift_per_sec;
        else gyroDrift -= drift_percentage * max_drift_per_sec;
        }

      lastTime = currentTime;
    }

    // error is greater than possible noise of magnetometer 
    
    
    // TODO: 1. Add second case of gyro drift logic 2. Get Accelerometer Values 3. Put code into functions

    // Second case: drift by percentage p of max_drift_per_sec, where p is how far into the bounds  (in percent) it is 

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
