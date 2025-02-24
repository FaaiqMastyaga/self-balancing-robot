#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {
  private:
    float kp, ki, kd;
    float input = 0, setpoint = 0;
    float err, sum_err = 0, deriv_error, prev_err = 0;
    float prev_time = 0;
    float output_min = -255, output_max = 255;

  public:
    // constructor
    PID();
    PID(float, float, float); // kp, ki, kd

    // set Parameters
    void setParams(float, float, float);  // kp, ki, kd
    // set PID Variable
    void setVar(float, float);  // input, setpoint
    // set output Range
    void setRange(float, float);
    
    // output calculation
    float compute();
    float compute(float, float);  // input, setpoint

    // get parameter
    float getKp();
    float getKi();
    float getKd();

    // get variable
    float getErr();
};

#endif