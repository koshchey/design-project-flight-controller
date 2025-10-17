// From CrazyFlie core, power_distribution_stock.c
// ESP Driver, motors.c

#include "driver/ledc.h"
#include "motors.h"
#include "stabiliser.h"

static bool isInit = false;
static bool isTimerInit = false;

uint32_t motor_ratios[] = {0, 0, 0, 0};
const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};

bool powerDistributionTest(void) {
  bool pass = true;
  pass &= motorsTest();
  return pass;
}

void powerDistribution(const control_t *control) {
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;

    motorPower.m1 = limitUint16(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitUint16(control->thrust - r - p - control->yaw);
    motorPower.m3 = limitUint16(control->thrust + r - p + control->yaw);
    motorPower.m4 = limitUint16(control->thrust + r + p - control->yaw);

    // motorPower.m1 = limitUint16(control->thrust);
    // motorPower.m2 = limitUint16(control->thrust);
    // motorPower.m3 = limitUint16(control->thrust);
    // motorPower.m4 = limitUint16(control->thrust);

    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }

    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
}

/**********************************************************************/

// thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust) {
    if (isInit) {
        uint16_t ratio;
        ratio = ithrust;
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
    }
}

bool motorsTest(void) {
    int i;
    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
          motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
          vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME_MS));
          motorsSetRatio(MOTORS[i], 0);
          vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME_MS));
    }
    return isInit;
}

/**********************************************************************/

void motorsInit(void) {
    int i;
    if (isInit) {
        return;
    }
    if (PWMTimerInit() != true) {
        return;
    }
    for (i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_channel_config(&motors_channel[i]);
    }
    isInit = true;
}

void motorsDeInit(void) {
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

bool PWMTimerInit(void) {
    if (isTimerInit) {
        return true;
    }

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, // resolution of PWM duty
        .freq_hz = 15000,	// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_0, // timer index
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = true;
        return true;
    }

    return false;
}

/**********************************************************************/

uint16_t limitUint16(int32_t value) {
  if(value > UINT16_MAX) {
    value = UINT16_MAX;
  } else if(value < 0) {
    value = 0;
  }
  return (uint16_t)value;
}

uint16_t motorsConvBitsTo16(uint16_t bits) {
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

uint16_t motorsConv16ToBits(uint16_t bits) {
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}