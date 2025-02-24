void setupMotor() {
  // Motor pins as outputs
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
}

void runLeftMotor(int pwm, int dir) {
  if (dir == -1) { // counter clockwise
    analogWrite(LEFT_RPWM, pwm);
    analogWrite(LEFT_LPWM, 0);
  } else if (dir == 1) { // clockwise
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, pwm);
  } else { // stop
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
  }
}

void runRightMotor(int pwm, int dir) {
  if (dir == -1) { // counter clockwise
    analogWrite(RIGHT_RPWM, pwm);
    analogWrite(RIGHT_LPWM, 0);
  } else if (dir == 1) { // clockwise
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, pwm);
  } else { // stop
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
  }
}