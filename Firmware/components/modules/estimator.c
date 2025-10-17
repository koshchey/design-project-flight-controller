// Ref: CrazyFlieCore, sensorfusion6.c 

#include <math.h>
#include "estimator.h"
#include "sensors.h"
#include "stabiliser.h"

static const char *TAG = "EST";

#define TWO_KP_DEF (2.0f * 0.6f) //(2.0f * 0.4f) // 2 * proportional gain (Kp)
#define TWO_KI_DEF (2.0f * 0.002f) //(2.0f * 0.001f) // 2 * integral gain (Ki)

static const float twoKp = TWO_KP_DEF;    
static const float twoKi = TWO_KI_DEF;    
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;  // integral error terms scaled by Ki


static float grav_x = 0.0f, grav_y = 0.0f, grav_z = 0.0f;
static float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f; 
static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
static float velz = 0.0f;

static const float pos_alpha = 0.90f;
static const float vel_alpha = 0.995f;
static const float vel_factor = 1.0f;

static float real_z_acce = 0.0f;
static float base_z = 1.0f; // 1g 
static bool is_acce_calibrated = false;

static bool use_tof = false;

void estimatorComplementary(sensor_data_t *sensor, attitude_t *attitude, float *vel_z, float *pos_z, const uint32_t tick) {
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    findAttitude(sensor->gyro.x, sensor->gyro.y, sensor->gyro.z, sensor->acce.x, sensor->acce.y, sensor->acce.z, ATTITUDE_UPDATE_DT);
    velocityEstimate(sensor->acce.x, sensor->acce.y, sensor->acce.z, vel_z, ATTITUDE_UPDATE_DT);
    getEulerRPY(attitude); 
  }

  // if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
  //   positionEstimate(vel_z, pos_z, 0, POS_UPDATE_DT);
  // }
}

/**********************************************************************/
void findAttitude(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  findQuaterion(gx, gy, gz, ax, ay, az, dt);
  
  // Estimate direction of gravity using quaternion rotation matrix, R(q):
  // [ 1 - 2(qy^2 + qz^2), 2(qx*qy - qw*qz),   2(qx*qz + qw*qy)   ]
  // [ 2(qx*qy + qw*qz),   1 - 2(qx^2 + qz^2), 2(qy*qz - qw*qx)   ]
  // [ 2(qx*qz - qw*qy),   2(qy*qz + qw*qx),   1 - 2(qx^2 + qy^2) ]
  // and point [0, 0, 1].T for gravity (R * p) 
  
  /* Estimate gravity */
  grav_x = 2 * (qx * qz - qw * qy);
  grav_y = 2 * (qw * qx + qy * qz);
  grav_z = qw * qw - qx * qx - qy * qy + qz * qz;

  /* Find gravity from sensors */
  if (!is_acce_calibrated) {
    base_z = (ax * grav_x + ay * grav_y + az * grav_z);
    is_acce_calibrated = true;
  }

  /* Find roll, pitch and yaw */
  if (grav_x>1) grav_x=1;
  if (grav_x<-1) grav_x=-1;

  yaw = atan2f(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) * 180 / (float)M_PI;

  // These come from rotation matrix for x-axis (Rx) (roll) and rotation matrix for y-axis (Ry) (pitch)
  // Rx(theta) * Ry(psi) * g
  // [     -sin(psi)     ] = -ax => psi = asin(ax)
  // [sin(theta)*cos(psi)] = -ay => cos(psi) = -ay/sin(theta) (1)
  // [cos(psi)*cos(theta)] = -az => from (1): -az = -ay/sin(theta) * cos(theta) -> theta = atan(ay/az)

  pitch = asinf(grav_x) * 180 / (float)M_PI;
  roll = atan2f(grav_y, grav_z) * 180 / (float)M_PI;
}

void velocityEstimate(float ax, float ay, float az, float *vel_z, float dt) {
  real_z_acce = (ax * grav_x + ay * grav_y + az * grav_z) - base_z;
  velz += deadband(real_z_acce, 0.06f) * dt * G;
  velz *= vel_alpha;
  *vel_z = velz;
}

/**********************************************************************/

void positionEstimate(float *vel_z, float *pos_z, float dist, float dt) {
  /* Estimate the position */
  const uint32_t max_sample_age = pdMS_TO_TICKS(50);
  float filtered_z = 0, estimated_z = 0;
  static float prev_estimated_z = 0.0f;

  bool sample_ok = false; // ((xTaskGetTickCount() - tof_data->timestamp) <= max_sample_age);

  if (sample_ok) {
    use_tof = true;
  }

  if (use_tof) {
    filtered_z = (pos_alpha * estimated_z + (1.0f - pos_alpha) * dist);
    estimated_z = filtered_z + (vel_factor * velz * dt);
  } 

  *pos_z = estimated_z;
  *vel_z = (estimated_z - prev_estimated_z) / dt;
  prev_estimated_z = estimated_z;
}

/**********************************************************************/

void getEulerRPY(attitude_t *attitude) {
  attitude->roll = roll; 
  attitude->pitch = pitch;
  attitude->yaw = yaw;
}

void getQuaternion(float *q_w, float *q_x, float *q_y, float *q_z) {
  // Save quaternion 
  *q_w = qw;
  *q_x = qx;
  *q_y = qy;
  *q_z = qz;
}

/**********************************************************************/

void findQuaterion(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  // gyro: gx, gy, gz
  // acce: ax, ay, az 

  /* Madgwick's implementation of Mayhony's AHRS algorithm (see: http://www.x-io.co.uk/open-source-ahrs-with-x-imu)*/
  // The code is from CrazyFlie

  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * (float)M_PI / 180 ; // Radians
  gy = gy * (float)M_PI / 180;
  gz = gz * (float)M_PI / 180;

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = qw;
  qb = qx;
  qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}

/**********************************************************************/

float invSqrt(float x) {
  return 1.0f/sqrtf(x);
  // float halfx = 0.5f * x;
  // float y = x;
  // long i = *(long*)&y;
  // i = 0x5f3759df - (i>>1);
  // y = *(float*)&i;
  // y = y * (1.5f - (halfx * y * y));
  // return y;
}

float deadband(float value, const float threshold) {
  if (fabsf(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}
