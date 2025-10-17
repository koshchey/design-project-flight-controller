idf_component_register(SRCS
                            "stabliser.c"
                            "sensors.c"
                            "estimator.c"
                            "i2c.c"
                            "websocket.c"
                            "controller.c"
                            "filter.c"
                            "motors.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_ledc esp_http_server esp_wifi nvs_flash esp_eth  protocol_examples_common)