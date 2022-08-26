// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16448_IMU.h>
#include <frc/Timer.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANSparkMax.h>

#define PI 3.141592

using namespace frc;

class SensorDriftModule {
 public:

  SensorDriftModule(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder); 

  void updateSensors();
  void outputMagnetometerNoise(double mag_heading);
  void outputMaxDriftPerSecond(double gyroVal);
  double sensorDriftAlgorithm(double gyroVal, double mag_heading, double lastTime);
  double driftSensor(double error_delta, double max_drift_per_second);
  void initializeSensors();
  void startTimer();

  // in inches
  const double rDistanceToCenter = 13.0; 
  const double lDistanceToCenter = 13.0;

  // gyro
  frc::ADIS16448_IMU* imu = new ADIS16448_IMU();

  rev::CANSparkMax* lMotor = nullptr;
  rev::CANSparkMax* rMotor = nullptr;

  // std::unique_ptr<rev::SparkMaxRelativeEncoder> lEncoder;
  // std::unique_ptr<rev::SparkMaxRelativeEncoder> rEncoder;

  rev::SparkMaxRelativeEncoder* lEncoder; 
  rev::SparkMaxRelativeEncoder* rEncoder;

  Timer* timer = new Timer(); 
  double driftLastTime = 0;
  //double lastTime = 0;
  double lastTimeGyro = 0;
  double lastTimeEncoder = 0;

  double gyro_drift = 0;
  double theta_drift = 0;


};

