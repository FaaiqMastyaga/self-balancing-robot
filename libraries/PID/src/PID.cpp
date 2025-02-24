#include <PID.h>

PID::PID(): kp(1), ki(0), kd(0) {}

PID::PID(float kpIn, float kiIn, float kdIn): kp(kpIn), ki(kiIn), kd(kdIn) {}

void PID::setParams(float kpIn, float kiIn, float kdIn) {
  kp = kpIn;
  ki = kiIn;
  kd = kdIn;
}

void PID::setVar(float inputIn, float setpointIn) {
  // set input & setpoint variable for output calculation
  input = inputIn;
  setpoint = setpointIn;
}

void PID::setRange(float outputMin, float outputMax) {
  output_min = outputMin;
  output_max = outputMax;
}

float PID::compute() {
  // calculate time
  long curr_time = micros();
  float delta_time = ((float)(curr_time-prev_time)) / 1.0e6;
  // calculate error
  err = setpoint - input;                                 // error
  sum_err = sum_err + err*delta_time;                     // integral error
  deriv_error = ((float)(err-prev_err)) / delta_time;     // derivative error
  // calculate output
  float output = kp*err + ki*sum_err + kd*deriv_error;
  output = constrain(output, output_min, output_max);
  // save last err
  prev_err = err;
  prev_time = curr_time;
  
  return output;
}

float PID::compute(float inputIn, float setpointIn) {
  this->setVar(inputIn, setpointIn);
  float output = this->compute();
  return output;
}

float PID::getKp() {
  return kp;
}

float PID::getKi() {
  return ki;
}

float PID::getKd() {
  return kd;
}

float PID::getErr() {
  return err;
}