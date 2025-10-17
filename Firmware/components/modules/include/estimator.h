#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include "sensors.h"
#include "stabiliser.h"
#include <stdint.h>

void estimatorComplementary(sensor_data_t *sensor, attitude_t *attitude, float *vel_z, float *pos_z, const uint32_t tick);
void findAttitude(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void velocityEstimate(float ax, float ay, float az, float *vel_z, float dt);
void positionEstimate(float *vel_z, float *pos_z, float dist, float dt);
void getEulerRPY(attitude_t *attitude);
void getQuaternion(float *q_w, float *q_x, float *q_y, float *q_z);
void findQuaterion(float gx, float gy, float gz, float ax, float ay, float az, float dt);
float invSqrt(float x);
float deadband(float value, const float threshold);

#endif