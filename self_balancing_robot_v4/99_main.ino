TaskHandle_t Task1;

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  Serial.println("Serial begin");

  xTaskCreatePinnedToCore(
        sendData, /* Function to implement the task */
        "Task1", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &Task1,  /* Task handle. */
        0); /* Core where the task should run */

  // Setup motors
  setupMotor();
  Serial.println("Setup Motor finished");

  // Setup encoder
  setupEncoder();
  Serial.println("Setup Encoder finished");

  // Setup MPU9250
  setupMpu();
  Serial.println("Setup MPU finished");

  Serial.println("Finish setup");
  Serial.println();

  AnglePID.setRange(-400, 400);
}

void loop() {
  if (mpuInterrupt) {
    // Reset interrupt flag
    mpuInterrupt = false;
    readDataMpu();
    
    float err_angle = pitch - target_angle;

    // set PID parameters (kp, ki, kd)
    if (abs(err_angle) < angle_tolerance) {
      AnglePID.setParams(3.25, 0, 0.125);
    }
    else {
      AnglePID.setParams(6.25, 0, 0.125);
    }

    rpm_setpoint = AnglePID.compute(pitch, target_angle);

    if (abs(err_angle) >= angle_fall) rpm_setpoint = 0;
  }
  // Serial.println("processing mpu");

  if (timer_triggered) {
    timer_triggered = false;

    float left_rpm = ((float) delta_left_pos / ppr) / ((sampling_time / 1.0e6) / 60.0);
    left_rpm_filtered = left_rpm * (1 - left_beta) + left_rpm_filtered * left_beta;

    float right_rpm = ((float) delta_right_pos / ppr) / ((sampling_time / 1.0e6) / 60.0);
    right_rpm_filtered = right_rpm * (1 - right_beta) + right_rpm_filtered * right_beta;
  }

  // compute PID motor
  left_rpm_setpoint = -rpm_setpoint;
  right_rpm_setpoint = rpm_setpoint;
  output_left = LeftMotorPID.compute(left_rpm_filtered, left_rpm_setpoint);
  output_right = RightMotorPID.compute(right_rpm_filtered, right_rpm_setpoint);
  // Serial.println("compute PID motor");

  int left_pwm = constrain(abs(output_left), 0, 255); 
  int right_pwm = constrain(abs(output_right), 0, 255);
  
  int left_dir = 1, right_dir = 1;
  if (output_left < 0) left_dir = -1;
  if (output_right < 0) right_dir = -1;
  
  runLeftMotor(left_pwm, left_dir);
  runRightMotor(right_pwm, right_dir);
}

void sendData(void* pvParameters) {
  // Setup WiFi
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  for(;;) {
    packData();
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}
void packData() {
  myData.pitch = pitch;
  myData.left_pos = left_pos;
  myData.right_pos = right_pos;
  myData.delta_left_pos = delta_left_pos;
  myData.delta_right_pos = delta_right_pos;
  myData.left_rpm_filtered = left_rpm_filtered;
  myData.right_rpm_filtered = right_rpm_filtered;
  myData.left_rpm_setpoint = left_rpm_setpoint;
  myData.right_rpm_setpoint = right_rpm_setpoint;
  myData.output_left = output_left;
  myData.output_right = output_right;
}