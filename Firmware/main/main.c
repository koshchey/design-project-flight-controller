#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "i2c.h"
#include "sensors.h"
#include "websocket.h"
#include "stabiliser.h"

void app_main(void)
{
    sensor_task_init();
    vTaskDelay(pdMS_TO_TICKS(2000));
    websocket_task_init();
    vTaskDelay(pdMS_TO_TICKS(2000));
    stabiliserTaskInit();
}
