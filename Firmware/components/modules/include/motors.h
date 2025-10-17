#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "stabiliser.h"

#define DEFAULT_IDLE_THRUST 0

#define NBR_OF_MOTORS 4
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#define MOTOR1_GPIO  4
#define MOTOR2_GPIO  33
#define MOTOR3_GPIO  32
#define MOTOR4_GPIO  25

#define MOT_PWM_CH1  4
#define MOT_PWM_CH2  5
#define MOT_PWM_CH3  6
#define MOT_PWM_CH4  7

#define MOTORS_PWM_BITS           LEDC_TIMER_8_BIT
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_TIM_BEEP_CLK_FREQ  4000000

#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

void motorsInit(void);
void motorsDeInit(void);
void motorsSetRatio(uint32_t id, uint16_t ithrust);
void powerDistribution(const control_t *control);
bool motorsTest(void);
bool powerDistributionTest(void);
void motorsInit(void);
void motorsDeInit(void);
bool PWMTimerInit(void);
uint16_t limitUint16(int32_t value);
uint16_t motorsConvBitsTo16(uint16_t bits);
uint16_t motorsConv16ToBits(uint16_t bits);

#endif