#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t bclk;
    gpio_num_t ws;
    gpio_num_t dout;
    gpio_num_t mclk;
    gpio_num_t mute;
    bool mute_active_high;
} audio_pins_t;

esp_err_t audio_start(const audio_pins_t *pins);
esp_err_t audio_write(const void *data, size_t len, TickType_t timeout_ticks, size_t *written);
uint32_t audio_get_sample_rate_hz(void);
esp_err_t audio_set_sample_rate_hz(uint32_t sample_rate_hz);
uint32_t audio_get_buffered_ms(void);

#ifdef __cplusplus
}
#endif
