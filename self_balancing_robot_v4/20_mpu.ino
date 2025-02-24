void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true; // Set interrupt flag
}

void setupMpu() {
  // Initialize MPU
  status = MPU.begin();
  if (!status) {
    Serial.println("MPU initialization unsuccessful");
    Serial.println("Check MPU wiring or try cycling power");
    ESP.restart();
  }

  // Enable the Data Ready Interrupt
  if (MPU.enableDataReadyInterrupt()) {
    Serial.println("Data Ready Interrupt enabled successfully!");
  } else {
    Serial.println("Failed to enable Data Ready Interrupt!");
  }

  // Attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);

  // Configure MPU settings
  MPU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  MPU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  MPU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
  // MPU.setSrd(9);

  // Calibrate MPU
  MPU.calibrateAccel();
  MPU.calibrateGyro();
  setGyroOffset();
}

void readDataMpu() {
  // Read sensor data
  MPU.readSensor();

  accX = MPU.getAccelX_mss();
  accY = MPU.getAccelY_mss();
  accZ = MPU.getAccelZ_mss();

  gyroX = MPU.getGyroX_rads();
  gyroY = MPU.getGyroY_rads();
  gyroZ = MPU.getGyroZ_rads();
  
  unsigned long curr_time = micros();
  float delta_time = (curr_time - prev_time) * 1.0e-6;
  prev_time = curr_time;

  // Calculate angles using accelerometer and gyroscope
  float pitch_acc = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / M_PI;
  float roll_acc = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / M_PI;

  float delta_pitch_gyro = (-gyroX * 180 / M_PI) * delta_time;
  float delta_roll_gyro = (gyroY * 180 / M_PI) * delta_time;
  float delta_yaw_gyro = (gyroZ * 180 / M_PI) * delta_time;

  pitch = alpha * (pitch + delta_pitch_gyro) + (1 - alpha) * pitch_acc;
  roll = alpha * (roll + delta_roll_gyro) + (1 - alpha) * roll_acc;
  yaw += delta_yaw_gyro;
}

void setGyroOffset() {
  float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

  for (int i = 0; i < samples; i++) {
    MPU.readSensor();

    gyroXOffset += MPU.getGyroX_rads();
    gyroYOffset += MPU.getGyroY_rads();
    gyroZOffset += MPU.getGyroZ_rads();
  }

  MPU.setGyroBiasX_rads(gyroXOffset / samples);
  MPU.setGyroBiasY_rads(gyroYOffset / samples);
  MPU.setGyroBiasZ_rads(gyroZOffset / samples);
}