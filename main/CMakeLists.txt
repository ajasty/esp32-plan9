idf_component_register(SRCS "base-c6.c" "lcd.c" "audio.c" "webserver.c" "ninep.c"
                    INCLUDE_DIRS "."
  REQUIRES esp_lcd esp_timer driver esp_driver_i2s esp_driver_ledc esp_adc esp_wifi esp_event esp_netif nvs_flash esp_http_server lwip
)
