#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include "filter.h"
#include "sensors.h"
#include "stabiliser.h"

// PID constants
#define PID_ROLL_RATE_KP  100.0
#define PID_ROLL_RATE_KI  0
#define PID_ROLL_RATE_KD  0
#define PID_ROLL_RATE_INT_LIMIT 33.3

#define PID_PITCH_RATE_KP  100.0
#define PID_PITCH_RATE_KI  0
#define PID_PITCH_RATE_KD  0
#define PID_PITCH_RATE_INT_LIMIT 33.3

#define PID_YAW_RATE_KP  50.0
#define PID_YAW_RATE_KI  0
#define PID_YAW_RATE_KD  0
#define PID_YAW_RATE_INT_LIMIT 166.7

#define PID_ROLL_KP  2
#define PID_ROLL_KI  0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INT_LIMIT 20.0

#define PID_PITCH_KP  2
#define PID_PITCH_KI  0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INT_LIMIT 20.0

#define PID_YAW_KP  2
#define PID_YAW_KI  0
#define PID_YAW_KD  0
#define PID_YAW_INT_LIMIT 360.0

#define DEFAULT_PID_INT_LIMIT 5000.0
#define DEFAULT_PID_OUT_LIMIT 0.0

#define ATT_LPF_FC 15.0f
#define ATT_LPF_EN true
#define ATT_RATE_LPF_FC 30.0f
#define ATT_RATE_LPF_EN true

#define THRUST_MIN 0
#define THRUST_MAX 65535

typedef struct
{
  float desired;
  float error;
  float prev_error;
  float integ;
  float deriv;
  float kp;
  float ki;
  float kd;
  float out_p;
  float out_i;
  float out_d;
  float int_limit;
  float out_limit;
  float dt;
  lpf_data d_filter;
  bool d_filter_en;
} PidObject;

void controllerInitPID(void);
void controllerPID(sensor_data_t *sensor, state_t *state, setpoint_t *setpoint, control_t *control, const uint32_t tick);
void pidInit(PidObject* pid, float desired, float kp, float ki, float kd, float dt, float fs, float fc, bool en_filter);
float pidUpdate(PidObject* pid, float measured, bool update_error);
void positionController(float *thrust, attitude_t *att_desired, setpoint_t *setpoint, state_t *state);
float angleLimit(float angle);
int16_t saturateSignedInt16(float in);
float constrain(float value, const float minVal, const float maxVal);
float scaleThrust(float actuator_thrust);
float scaleAngle90(float setpoint);
float scaleAngle180(float setpoint);

#endif