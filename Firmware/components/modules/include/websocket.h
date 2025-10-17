#ifndef WS_H_
#define WS_H_

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "esp_http_server.h"

// hd = Handle (server)
// fd = socket descriptor (client) 
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
}; // old version

typedef void (*data_read_function)(char*, uint8_t);

struct async_send_arg {
    httpd_handle_t hd;
    int fd;
    void *data;
    data_read_function funct;
};

void websocket_task_init(void);
bool websocket_is_init(void);
void get_imu_data(char *data, uint8_t len);
void get_attitude_data(char *data, uint8_t len);
esp_err_t trigger_async_send_imu(httpd_handle_t handle, httpd_req_t *req);
esp_err_t handle_ws_req(httpd_req_t *req);
httpd_handle_t start_webserver(void);

#endif