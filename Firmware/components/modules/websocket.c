
// Ref: ESP-IDF v5.5 "ws_echo_server"
// Ref: https://esp32tutorials.com/esp32-esp-idf-websocket-web-server/

#include "sensors.h"
#include "stabiliser.h"
#include "websocket.h"

static const char *TAG = "WEB";

static httpd_handle_t server = NULL;
static sensor_data_t sensor_data = {0};
static socket_data_t display = {0};
static bool websocket_init = false;

void get_imu_data(char *data, uint8_t len) {
    readIMUData(&sensor_data);
    snprintf(data, len, "imu%06.2f%06.2f%06.2f%06.2f%06.2f%06.2f",
             sensor_data.acce.x, sensor_data.acce.y, sensor_data.acce.z,
             sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z);
}

void get_display(char *data, uint8_t len) {
    readSocketData(&sensor_data, &display);
    snprintf(data, len, "rpy%06.2f%06.2f%06.2f%06.2f%06.2f%06.2f%06.2f%06.2f",
             display.attitude.roll, display.attitude.pitch, display.attitude.yaw, display.z,
             (float)display.control.roll, (float)display.control.pitch, (float)display.control.yaw, display.control.thrust);
}

void send_setpoint(char *data, uint8_t len) {
    if (len < sizeof(setpoint_t)) return;
    setpoint_t sp;
    memcpy(&sp, data, sizeof(setpoint_t));
    sp.timestamp = xTaskGetTickCount();
    setSetpoint(&sp);
}

/**************************************************************/

void async_send(void *arg) {
    struct async_send_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;

    data_read_function funct = resp_arg->funct; 
    char data[255] = {0};
    funct(data, sizeof(data)-1); // pass the correct data function

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req, data_read_function funct) {
    struct async_send_arg *resp_arg = malloc(sizeof(struct async_send_arg));
    
    if (resp_arg == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    resp_arg->funct = funct; 
    esp_err_t ret = httpd_queue_work(handle, async_send, resp_arg);
    
    if (ret != ESP_OK) {
        free(resp_arg);
    }
    
    return ret;
}

/**************************************************************/
/* Handle messages being recieved from clients */
esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // Set max_len = 0 to get the frame len
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    
    // ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        // ws_pkt.len + 1 is for NULL termination as we are expecting a string 
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        // Set max_len = ws_pkt.len to get the frame payload 
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        // ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }
    // ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);

    /* Handle messages */
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        if (strcmp((char*)ws_pkt.payload, "request imu") == 0) {
            free(buf);
            return trigger_async_send(req->handle, req, get_imu_data);
        } else if (strcmp((char*)ws_pkt.payload, "request rpy") == 0) {
            free(buf);
            ret = trigger_async_send(req->handle, req, get_display);
            return (trigger_async_send(req->handle, req, get_imu_data) || ret); // changes
        }
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_BINARY) {
        if (strncmp((char*)ws_pkt.payload, "set", 3) == 0) {
            // ESP_LOGI(TAG, "Setpoint received");
            if (ws_pkt.len >= 3 + sizeof(setpoint_t)) {
                setpoint_t sp;
                memcpy(&sp, ws_pkt.payload + 3, sizeof(setpoint_t));
                send_setpoint((char*)&sp, sizeof(setpoint_t));
            } 
        }
    }
    
    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    }
    free(buf);

    return ret;
}

/**************************************************************/

const httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = handle_ws_req,
        .user_ctx   = NULL,
        .is_websocket = true
};

httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        return server;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void websocket_task_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_err_t ret = example_connect();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Server failed to connect");
        return; 
    }

    server = start_webserver();

    websocket_init = true;
}

bool websocket_is_init() {
    return websocket_init;
}