#include <MPU9250.h>
#include <PID.h>
#include <esp_now.h>
#include <WiFi.h>

hw_timer_t *timer = NULL;  // Declare the timer object
bool timer_triggered = false;

// interrupt
portMUX_TYPE mpu_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE left_enc_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE right_enc_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

// PID
// PID objectPID(kp, ki, kd);
PID LeftMotorPID(1.8, 7.0, 0);
PID RightMotorPID(1.8, 7.0, 0);
PID AnglePID(3.0, 0.0, 0.0);

const float target_angle = 2.35;
const float angle_tolerance = 5.0;
const float angle_fall = 75.0;

// Encoder
#define RIGHT_ENCA 34
#define RIGHT_ENCB 35
#define LEFT_ENCA 36
#define LEFT_ENCB 39

const int ppr = 330;
const int sampling_time = 10000; // (us)
long last_sample_time = 0;

volatile int left_pos = 0;
int delta_left_pos = 0;
float output_left;
float left_rpm_filtered = 0;
const float left_beta = 0.92; // lowpass filter constant

volatile int right_pos = 0;
int delta_right_pos = 0;
float output_right;
float right_rpm_filtered = 0;
const float right_beta = 0.92; // low pass filter constant

// Motor
#define RIGHT_RPWM 26
#define RIGHT_LPWM 27
#define LEFT_RPWM 32
#define LEFT_LPWM 33

int pwm;
float rpm_setpoint, left_rpm_setpoint, right_rpm_setpoint;

// MPU
#define MPU_INT 18

MPU9250 MPU(Wire, 0x68);
int status;
volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high

const float alpha = 0.96; // Complementary filter constant
const int samples = 100;  // Samples for gyro offset calibration

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch = 0, roll = 0, yaw = 0;

unsigned long prev_time = 0;

// ESP NOW
// a0:b7:65:69:84:c8
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0x69, 0x84, 0xC8};

typedef struct struct_message {
  float pitch;
  float left_pos, right_pos, delta_left_pos, delta_right_pos;
  float left_rpm_filtered, right_rpm_filtered;
  float left_rpm_setpoint, right_rpm_setpoint;
  float output_left, output_right;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}