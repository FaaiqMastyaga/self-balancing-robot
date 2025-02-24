#include <MovingAverage.h>
#include <PID.h>

#define RIGHT_RPWM 26
#define RIGHT_LPWM 27
#define LEFT_RPWM 32
#define LEFT_LPWM 33

#define RIGHT_ENCA 4
#define RIGHT_ENCB 5
#define LEFT_ENCA 4
#define LEFT_ENCB 5

MovingAverage<float, float> filter;

portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;

const int ppr = 330;

volatile int pos = 0;

int prev_pos = 0;
long prev_time = 0;

const int sampling_interval = 10000; // 10 ms
long last_sample_time = 0;

// PID
const float kp = 0.85, ki = 10.0, kd = 0;
float setpoint_velocity;
float err, err_int = 0, dedt;
float u;

PID Motor(kp, ki, kd);

void IRAM_ATTR readEncoder() {
  portENTER_CRITICAL_ISR(&enc_mux);
  if (digitalRead(RIGHT_ENCB)) pos++;
  else pos--;
  portEXIT_CRITICAL_ISR(&enc_mux);
}

void setup() {
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  pinMode(RIGHT_ENCA, INPUT);
  pinMode(RIGHT_ENCB, INPUT);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readEncoder, RISING);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readEncoder, RISING);

  filter.begin();
}

void loop() {
  // Read user input only when available
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n'); // Read input until newline
    inputString.trim(); // Remove whitespace/newline

    // Check if the input is numeric
    if (inputString.length() > 0 && isNumeric(inputString)) {
      setpoint_velocity = inputString.toFloat(); // Convert to float
    }
  }

  if (micros() - last_sample_time >= sampling_interval) {
    last_sample_time = micros();
    
    int curr_pos;
    portENTER_CRITICAL_ISR(&enc_mux);
    curr_pos = pos;
    portEXIT_CRITICAL_ISR(&enc_mux);

    long curr_time = micros();
    float delta_time = ((float)(curr_time - prev_time)) / 1.0e6;
    float velocity = ((float)(curr_pos - prev_pos) / ppr) / delta_time * 60.0;
    filter.add(velocity);
    float velocity_filtered = filter.readAverage(10);

    prev_pos = curr_pos;
    prev_time = curr_time;

    u = Motor.compute(velocity_filtered, setpoint_velocity);

    Serial.print("setpoint_velocity:");
    Serial.print(setpoint_velocity);
    Serial.print("  ");
    // Serial.print("velocity:");
    // Serial.print(velocity);
    // Serial.print("  ");
    Serial.print("velocity_filtered:");
    Serial.print(velocity_filtered);
    Serial.print("  ");
    Serial.print("u:");
    Serial.print(u);

    Serial.println();
  }

  int pwm = constrain(abs(u), 0, 255);
  int dir = 1;
  if (u < 0) dir = -1;
  runMotor(pwm, dir);
}

void runMotor(int pwm, int dir) {
  if (dir == -1) { // counter clockwise
    analogWrite(LEFT_RPWM, pwm);
    analogWrite(LEFT_LPWM, 0);
    // analogWrite(RIGHT_RPWM, pwm);
    // analogWrite(RIGHT_LPWM, 0);
  } else if (dir == 1) { // clockwise
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, pwm);
    // analogWrite(RIGHT_RPWM, 0);
    // analogWrite(RIGHT_LPWM, pwm);
  } else {
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
    // analogWrite(RIGHT_RPWM, 0);
    // analogWrite(RIGHT_LPWM, 0);
  }
}

// Function to check if input is numeric
bool isNumeric(String str) {
  for (byte i = 0; i < str.length(); i++) {
    if (!isDigit(str[i]) && str[i] != '.' && str[i] != '-') {
      return false; // Return false if any non-numeric character is found
    }
  }
  return true;
}