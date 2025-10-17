#include "stabiliser.h"
#include "websocket.h"
#include "sensors.h"
#include "estimator.h"
#include "controller.h"
#include "motors.h"

#define TESTING false
#define YAW_OFF false

static const char *TAG = "STAB";

#define TIMEOUT_STABILIZE pdMS_TO_TICKS(1000)

static TaskHandle_t stabiliser_task_handle = NULL;

static QueueHandle_t socket_data_queue = NULL;
static QueueHandle_t setpoint_queue = NULL;

static uint32_t lastWakeTime;
static const TickType_t xFrequency = 1;

static state_t state = {0};
static setpoint_t setpoint = {0};
static control_t control = {0};
static sensor_data_t sensor_data = {0};
static socket_data_t socket_data = {0};
static tof_data_t tof_data = {0}; //(?)

const static control_t control_reset = {0};

static bool websocket_disconnect = false;

/**************************************************************/

static void stabiliser_task(void* param) {
  uint32_t tick; 

  if (!sensorsIsInit() && !websocket_is_init()) {
    vTaskDelay(portMAX_DELAY);
  }
  ESP_LOGI(TAG, "Ready to fly!");

  lastWakeTime = xTaskGetTickCount(); // not using this right now but may use later
  tick = 1;

  while(1) {
    waitSensorData();
    readIMUData(&sensor_data);
    
    /* Get state estimate */
    // Also position estimate (to-do)
    // wait_for_vlx_data();
    estimatorComplementary(&sensor_data, &state.attitude, &state.vz, &state.z, tick);

    /* Update display data */
    sendDisplaySocket();
    
    /* Get set point */
    getSetpoint(&setpoint); 
    stabiliserLog(tick);

    /* PID controller */
    controllerPID(&sensor_data, &state, &setpoint, &control, tick);

    /******* Testing ********/ 
    control.yaw = 0;
    
    /* Send control points to motors */
    if (setpoint.enable && !websocket_disconnect) {
      powerDistribution(&control); 
    } else {
      powerDistribution(&control_reset); 
    }
    
    // Update tick
    tick++;
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

bool stabiliserTest(void) {
  bool pass = true;
  pass &= powerDistributionTest(); 
  return pass;
}

/**************************************************************/

void getSetpoint(setpoint_t *setpoint) {
  uint32_t now = xTaskGetTickCount();

  if(xQueuePeek(setpoint_queue, (void *)setpoint, 0) == pdTRUE) {
    // setpoint->attitude.roll = 0;
    // setpoint->attitude.pitch = 0;
    // setpoint->attitudeRate.yaw = 0;

    if (((now - setpoint->timestamp) > TIMEOUT_STABILIZE) && !websocket_disconnect && !TESTING) {
      // To-do: might need to change how the timeout works
      setpoint->enable = false;
      websocket_disconnect = true; 
      ESP_LOGE(TAG, "Websocket timestamp too old, disabling motors");
      return;
    }  
  }

  websocket_disconnect = false; 
}

void setSetpoint(setpoint_t *setpoint) {
  xQueueOverwrite(setpoint_queue, setpoint);
}

void sendDisplaySocket() {
  socket_data.attitude = state.attitude;
  socket_data.z = state.z;
  socket_data.control = control;
  xQueueOverwrite(socket_data_queue, (void *)&socket_data);
}

void readSocketData(sensor_data_t *sensor, socket_data_t *socket_data) {
  xQueuePeek(socket_data_queue, (void *)socket_data, 0);
}

/**************************************************************/

void stabiliserLog(const uint32_t tick) {
  if (RATE_DO_EXECUTE(LOG_RATE, tick)) {
      ESP_LOGI(TAG, "tick=%lu", (unsigned long)tick);
      ESP_LOGI(TAG, "sensor: gyro[%.2f %.2f %.2f] acce[%.2f %.2f %.2f]",
        sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z,
        sensor_data.acce.x, sensor_data.acce.y, sensor_data.acce.z);
      ESP_LOGI(TAG, "state: att[roll=%.2f pitch=%.2f yaw=%.2f] vz=%.2f z=%.2f",
        state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
        state.vz, state.z);
      ESP_LOGI(TAG, "setpoint: att[roll=%.2f pitch=%.2f yaw=%.2f] enable %d "
                    "attRate[roll=%.2f pitch=%.2f yaw=%.2f] thrust=%.2f",
        setpoint.attitude.roll, setpoint.attitude.pitch, setpoint.attitude.yaw, setpoint.enable,
        setpoint.attitudeRate.roll, setpoint.attitudeRate.pitch, setpoint.attitudeRate.yaw, setpoint.thrust);
      ESP_LOGI(TAG, "control: roll=%d pitch=%d yaw=%d thrust=%.2f",
        control.roll, control.pitch, control.yaw, control.thrust);
  }
}

void stabiliserTaskInit(void) {
    socket_data_queue = xQueueCreate(1, sizeof(socket_data_t));
    setpoint_queue = xQueueCreate(1, sizeof(setpoint_t));

    /* Init */
    motorsInit();
    PWMTimerInit();
    ESP_LOGI(TAG, "Motor GPIO intialised");

    controllerInitPID();
    ESP_LOGI(TAG, "PID controller intialised");

    /* Tests */
    if (!stabiliserTest()) {
      ESP_LOGE(TAG, "Stabiliser tests failed");
    } else {
      ESP_LOGI(TAG, "Stabiliser tests passed");
    }

    /* Task */
    xTaskCreate(stabiliser_task, "stabiliser task", 3*1024, NULL, 5, &stabiliser_task_handle);
    ESP_LOGI(TAG, "Stabiliser task intialised");
}
