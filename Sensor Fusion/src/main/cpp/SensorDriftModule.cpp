// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SensorDriftModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


SensorDriftModule::SensorDriftModule(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder): lEncoder{lEncoder}, rEncoder{rEncoder} {
    // this->lEncoder = lEncoder;
    // this->rEncoder = rEncoder;
    
    // this->lMotor = lMotor;
    // this->rMotor = rMotor;

    // rev::SparkMaxRelativeEncoder lEncoder_og = lMotor->GetEncoder();
    // rev::SparkMaxRelativeEncoder rEncoder_og = rMotor->GetEncoder();

    // lEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(lEncoder_og);
    // rEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(rEncoder_og);



    // rev::SparkMaxRelativeEncoder lEncoder_og = lMotor->GetEncoder();
    // rev::SparkMaxRelativeEncoder rEncoder_og = rMotor->GetEncoder();

    // lEncoder = &lEncoder_og;
    // rEncoder = &rEncoder_og;

}

void SensorDriftModule::updateSensors() {

  // Part 1: Reading/Calculating Sensor Data (3 Sensors)

  // Sensor 1: Magnetometer
  units::magnetic_field_strength::tesla_t mag_x_native = imu->GetMagneticFieldX();
  units::magnetic_field_strength::tesla_t mag_y_native = imu->GetMagneticFieldY();

  double mag_x = mag_x_native.value();
  double mag_y = mag_y_native.value();

  // mag_heading is the heading from the magnetometer, where 0 is North
  double mag_heading = ((atan2(mag_y, mag_x) * 180) / PI);

  /* og_mag_heading represents the original magnetometer heading, which is added to the gyro value so 
     that sensor readings match regardless of starting orientation */
  static double og_mag_heading = mag_heading;
  frc::SmartDashboard::PutNumber("og mag heading", og_mag_heading);
  frc::SmartDashboard::PutNumber("mag heading", mag_heading);

  // Sensor 2: Motor Encoders

  rev::SparkMaxRelativeEncoder lEncoder_obj = *lEncoder;
  rev::SparkMaxRelativeEncoder rEncoder_obj = *rEncoder;

  double lRotations = lEncoder_obj.GetPosition(); 
  double rRotations = rEncoder_obj.GetPosition();
  double theta = (rRotations - lRotations) / (rDistanceToCenter + lDistanceToCenter);
  theta = theta * 180 / PI;

  // converts to bounds (-180, 180)
  theta = fmod(theta + og_mag_heading + theta_drift, 360);
  if (theta < -180) theta += 360;
  else if (theta > 180) theta -= 360; 

  frc::SmartDashboard::PutNumber("drifted theta", theta);

  // Sensor 3: Gyro
  // gyroVal is the raw value from the gyro
  double gyro_val = imu->GetAngle().value();
  frc::SmartDashboard::PutNumber("raw gyro value", gyro_val);

  double gyro_adjusted_val = fmod((-gyro_val + gyro_drift + og_mag_heading), 360); // gyro val is negative because imu is backwards
  if (gyro_adjusted_val < -180) gyro_adjusted_val += 360;
  else if (gyro_adjusted_val > 180) gyro_adjusted_val -= 360; 
  frc::SmartDashboard::PutNumber("drifted gyro val", gyro_adjusted_val);

  // functions used to find the noise of the magnetometer and max drift per second of the gyro
  // outputMagnetometerNoise(mag_heading);
  // outputMaxDriftPerSecond(gyro_val);

  // Part 2: Drifting Gyro to match Magnetometer heading

  gyro_drift += sensorDriftAlgorithm(gyro_adjusted_val, mag_heading, lastTimeGyro);
  theta_drift += sensorDriftAlgorithm(theta, mag_heading, lastTimeEncoder);
}


double SensorDriftModule::sensorDriftAlgorithm(double gyro_adjusted_val, double mag_heading, double lastTime) {

  

  // observed noise of magnetometer and max drift of gyro 
  double const noise_p = 0.02; 
  double const max_drift_per_sec = 0.4; 

  double error_delta = mag_heading - gyro_adjusted_val;
  double error_bound = noise_p * fabs(mag_heading);

  double currentTime = timer->GetFPGATimestamp().value();
  double elapsedTime = currentTime - lastTime;

  if (elapsedTime > 1) {

    if (lastTime == lastTimeGyro) lastTimeGyro = currentTime;
    else lastTimeEncoder = currentTime;

    // drifts gyro value to magnetometer heading at max drift per sec 
    if (fabs(error_delta) > error_bound) {
      //gyro_drift += driftSensor(error_delta, max_drift_per_sec);
      return driftSensor(error_delta, max_drift_per_sec);
    }    

    // drifts gyro value to magnetometer heading relative to how far in the bounds the gyro value is
    else {
      double drift_percentage = 1 - ((error_bound - error_delta) / error_bound);
      //gyro_drift += driftSensor(error_delta, max_drift_per_sec * drift_percentage);
      return driftSensor(error_delta, max_drift_per_sec * drift_percentage);
    }
    
    // if initial values are 0, function needs to be called with gyro first
    

    //lastTime = currentTime;
  }

  return 0.0;
}


// Decides whether to add or subtract the drift amount depending on the readings of the sensors. Also adresses the wrap-around cases.  
double SensorDriftModule::driftSensor(double error_delta, double drift) {
  if (error_delta > 0) {
    // addresses the case where gyro is negative and magnetometer heading is positive
    if (fabs(error_delta) > 180) {
      return -drift;
    }
    return drift;

  } else {
    // addresses the where magnetometer heading is negative and gyro is positive
    if (fabs(error_delta) > 180) {
      return drift;
    }
    return -drift;
  }
}


void SensorDriftModule::outputMagnetometerNoise(double mag_heading) {

    static double og_val = mag_heading;
    static double max_noise = 0;

    if (fabs(mag_heading - og_val) > max_noise) max_noise = fabs(mag_heading - og_val);
    
    frc::SmartDashboard::PutNumber("max noise", max_noise);

    // represents the percentage of the magnetometer heading that is noise
    frc::SmartDashboard::PutNumber("noise percentage", (max_noise / (og_val + max_noise)) * 100);
}


void SensorDriftModule::outputMaxDriftPerSecond (double gyroVal) {
  static double maxGyroDriftPerSec = 0;
  static double previousDrift = gyroVal;
  double driftCurrentTime = timer->GetFPGATimestamp().value();
  double driftElapsedTime = driftCurrentTime - driftLastTime;

    if (driftElapsedTime > 1) {
      if (fabs(previousDrift - gyroVal) > maxGyroDriftPerSec){
        maxGyroDriftPerSec = fabs(previousDrift - gyroVal);
      }
      previousDrift = gyroVal;
      driftLastTime = driftCurrentTime;
    }

    frc::SmartDashboard::PutNumber("maxGryoDriftPerSec", maxGyroDriftPerSec);
}


void SensorDriftModule::initializeSensors() {

    rev::SparkMaxRelativeEncoder lEncoder_obj = *lEncoder;
    rev::SparkMaxRelativeEncoder rEncoder_obj = *rEncoder;


    lEncoder_obj.SetPosition(0);
    rEncoder_obj.SetPosition(0);

    // 1 / (6.4 / (4 * pi)) --> 1.96
    // 6.4 is gear ratio and 4pi is circumference
    lEncoder_obj.SetPositionConversionFactor(1.96);
    rEncoder_obj.SetPositionConversionFactor(1.96);

    imu->Reset();
    timer->Reset();
}

void SensorDriftModule::startTimer() {
    timer->Start();
}


