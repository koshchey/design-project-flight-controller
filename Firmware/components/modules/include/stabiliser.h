#ifndef STABILISER_H_
#define STABILISER_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sensors.h"

// Ref: CrazyFlie
// Used so that not every part of the stabiliser updates at the main loop rate of 1kHz
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#define LOG_RATE 10

typedef struct {
  float roll;
  float pitch;
  float yaw;
} attitude_t;

typedef struct {
  attitude_t attitude; 
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
} state_t;

// make sure the compiler doesn't add extra padding
typedef struct __attribute__((packed)) {
  bool enable;
  uint32_t timestamp;
  attitude_t attitude; 
  attitude_t attitudeRate; 
  float thrust; 
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
} setpoint_t;

typedef struct {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
} control_t;

typedef struct {
  attitude_t attitude; 
  float z; 
  control_t control;
} socket_data_t;


void stabiliserTaskInit(void);
void stabiliserLog(const uint32_t tick);
bool stabiliserTest(void);
void getSetpoint(setpoint_t *setpoint);
void setSetpoint(setpoint_t *setpoint);
void readSocketData(sensor_data_t *sensor, socket_data_t *display);
void sendDisplaySocket(void);

#endif