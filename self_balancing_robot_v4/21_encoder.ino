void IRAM_ATTR readLeftEncoder() {
  portENTER_CRITICAL_ISR(&left_enc_mux);
  if (digitalRead(LEFT_ENCB)) left_pos++;
  else left_pos--;
  portEXIT_CRITICAL_ISR(&left_enc_mux);
}

void IRAM_ATTR readRightEncoder() {
  portENTER_CRITICAL_ISR(&right_enc_mux);
  if (digitalRead(RIGHT_ENCB)) right_pos++;
  else right_pos--;
  portEXIT_CRITICAL_ISR(&right_enc_mux);
}

void IRAM_ATTR calculateRpm() {
  // Get encoder positions safely
  portENTER_CRITICAL_ISR(&timer_mux);
  delta_left_pos = left_pos;
  delta_right_pos = right_pos;
  left_pos = 0;
  right_pos = 0;
  timer_triggered = true;
  portEXIT_CRITICAL_ISR(&timer_mux);
}

void setupEncoder() {
  pinMode(LEFT_ENCA, INPUT);
  pinMode(LEFT_ENCB, INPUT);
  pinMode(RIGHT_ENCA, INPUT);
  pinMode(RIGHT_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readRightEncoder, RISING);

  // Initialize hardware timer
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &calculateRpm);
  timerAlarm(timer, sampling_time, true, 0);
}