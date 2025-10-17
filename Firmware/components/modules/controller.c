// Ref: CrazyFlie PID code : ) 

#include "stabiliser.h"
#include "controller.h"
#include "filter.h"
#include "sensors.h"
#include "estimator.h"

static const char *TAG = "PID";
static const bool isManualControl = true;

static attitude_t att_desired = {0};
static attitude_t att_rate_desired = {0};
static attitude_t att_output = {0};
static float actuator_thrust = 0;

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidPosX;
PidObject pidPosY;
PidObject pidPosZ;

float thrust_desired = 0; 
float pid_roll = 0, pid_pitch = 0, pid_yaw = 0; 
float pid_is_init = false; 

void controllerInitPID() {
  /* Init all the PID objects */
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
          ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_RATE_LPF_EN);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
          ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_RATE_LPF_EN);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
          ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_RATE_LPF_EN);
  
  pidRollRate.int_limit = PID_ROLL_RATE_INT_LIMIT; 
  pidPitchRate.int_limit = PID_PITCH_RATE_INT_LIMIT;
  pidYawRate.int_limit = PID_YAW_RATE_INT_LIMIT;

  pidInit(&pidRoll, 0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD, ATTITUDE_UPDATE_DT,
          ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_LPF_EN);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, ATTITUDE_UPDATE_DT,
          ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_LPF_EN);
  pidInit(&pidYaw, 0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD, ATTITUDE_UPDATE_DT,
          ATTITUDE_RATE, ATT_RATE_LPF_FC, ATT_LPF_EN);

  pidRoll.int_limit = PID_ROLL_INT_LIMIT; 
  pidPitch.int_limit = PID_PITCH_INT_LIMIT;
  pidYaw.int_limit = PID_YAW_INT_LIMIT;

  //To-Do: Add position controller

  pid_is_init = true;
}

void controllerPID(sensor_data_t *sensor, state_t *state, setpoint_t *setpoint, control_t *control, const uint32_t tick) {
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      att_desired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
      att_desired.yaw = angleLimit(att_desired.yaw); 
  }

  /* Position controller */
  // This gets the thrust from the z velocity (outer-outer loop)
  // To-do: consider x, y tuning
  // (which updates the desired attitude from something other than zero (stable hover))
  // if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
  //   positionController(&actuator_thrust, &att_desired, setpoint, state);
  // }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    actuator_thrust = setpoint->thrust; // Use this for now 
    att_desired.roll = setpoint->attitude.roll;
    att_desired.pitch = setpoint->attitude.pitch; 

    /* Attitude Control */
    /* PID for attitude */
    // This gets the attitude rate from the attitude (outer loop)
    pidRoll.desired = att_desired.roll;
    att_rate_desired.roll = pidUpdate(&pidRoll, state->attitude.roll, true); 

    pidPitch.desired = att_desired.pitch;
    att_rate_desired.pitch = pidUpdate(&pidPitch, state->attitude.pitch, true); 

    float yaw_error;
    yaw_error = setpoint->attitude.yaw - state->attitude.yaw;
    yaw_error = angleLimit(yaw_error);
    pidYaw.error = yaw_error;
    att_rate_desired.yaw = pidUpdate(&pidYaw, state->attitude.yaw, false);
    
    /* Attitude Rate Control */
    // From the attitude rate get an output to the motor! (inner loop)
    pidRollRate.desired = att_rate_desired.roll;
    att_output.roll = pidUpdate(&pidRollRate, sensor->gyro.x, true);
    pidPitchRate.desired = att_rate_desired.pitch;
    att_output.pitch = pidUpdate(&pidPitchRate, sensor->gyro.y, true);
    pidYawRate.desired = att_rate_desired.yaw;
    att_output.yaw = pidUpdate(&pidYawRate, sensor->gyro.z, true);

    control->roll = saturateSignedInt16(att_output.roll);
    control->pitch = saturateSignedInt16(att_output.pitch);
    control->yaw = saturateSignedInt16(att_output.yaw);

    if (isManualControl) {
      control->thrust = scaleThrust(actuator_thrust);
      // control->roll = scaleAngle90(setpoint->attitude.roll);
      // control->pitch = scaleAngle90(setpoint->attitude.pitch); 
      // control->yaw = scaleAngle180(att_desired.yaw); 
    } else {
      control->thrust = actuator_thrust;
    } 

    if (RATE_DO_EXECUTE(LOG_RATE, tick)) {
      ESP_LOGI(TAG, "tick=%lu", (unsigned long)tick);
      ESP_LOGI(TAG, "att_des: [roll=%.2f pitch=%.2f yaw=%.2f] thrust=%.2f",
        att_desired.roll, att_desired.pitch, att_desired.yaw, actuator_thrust);
      ESP_LOGI(TAG, "att_rate_des: [roll=%.2f pitch=%.2f yaw=%.2f]",
       att_rate_desired.roll, att_rate_desired.pitch, att_rate_desired.yaw);
      ESP_LOGI(TAG, "thrust: [act=%.2f out=%.2f]",
      actuator_thrust, control->thrust);
    }
  }
}

/******************************************************************/

void positionController(float *thrust, attitude_t *att_desired, setpoint_t *setpoint, state_t *state) {
  // Not finished
  thrust_desired = setpoint->thrust;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->vx;
  float bodyvy = setpoint->vy;
  setpoint->vx = bodyvx * cosyaw - bodyvy * sinyaw;
  setpoint->vy = bodyvy * cosyaw + bodyvx * sinyaw;

  pidPosZ.desired = setpoint->z;
  setpoint->vz = pidUpdate(&pidPosZ, state->z, true);
}

/******************************************************************/

// From CrazyFlie core, PID.C
void pidInit(PidObject* pid, float desired, float kp, float ki, float kd, float dt, float fs, float fc, bool en_filter) {
  pid->desired = desired;
  pid->error = 0;
  pid->prev_error = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->int_limit = 5000.0;
  pid->out_limit = 0.0;
  pid->dt = dt;
  pid->d_filter_en = en_filter;

  if (pid->d_filter_en) {
    init_lpf(&pid->d_filter, fs, fc);
  }
}

float pidUpdate(PidObject* pid, float measured, bool update_error) {
  float output = 0.0f;

  if (update_error) {
    pid->error = pid->desired - measured;
  }

  /* Proportional */
  pid->out_p = pid->kp * pid->error;
  output += pid->out_p;

  /* Derivative */
  float deriv = (pid->error - pid->prev_error) / pid->dt;
  if (pid->d_filter_en) {
    pid->deriv = apply_lpf(&pid->d_filter, deriv);
  } else {
    pid->deriv = deriv;
  }

  if (isnan(pid->deriv)) {
    pid->deriv = 0;
  }
  
  pid->out_d = pid->kd * pid->deriv;
  output += pid->out_d;

  /* Integral */
  pid->integ += pid->error * pid->dt;

  // Constrain the integral (unless the int_limit is zero)
  if(pid->int_limit != 0) {
    pid->integ = constrain(pid->integ, -pid->int_limit, pid->int_limit);
  }

  pid->out_i = pid->ki * pid->integ;
  output += pid->out_i;

  // Constrain the total PID output (unless the out_limit is zero)
  if(pid->out_limit != 0) {
    output = constrain(output, -pid->out_limit, pid->out_limit);
  }

  pid->prev_error = pid->error;

  return output;
}

/******************************************************************/
float scaleThrust(float actuator_thrust) {
    // for actuator_thrust between 0-100
    if (actuator_thrust < 0.0f) actuator_thrust = 0.0f;
    if (actuator_thrust > 100.0f) actuator_thrust = 100.0f;
    // Scale 0-100 → THRUST_MIN-THRUST_MAX
    return (actuator_thrust / 100.0f) * (float)(THRUST_MAX - THRUST_MIN) + THRUST_MIN;
}

float scaleAngle90(float setpoint) {
    return (setpoint/90.0f) * (float)(INT16_MAX);
}

float scaleAngle180(float setpoint) {
    return (setpoint/180.0f) * (float)(INT16_MAX);
}

// Some helpers from CrazyFlie : )
float angleLimit(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

int16_t saturateSignedInt16(float in)
{
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}
