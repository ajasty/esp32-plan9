#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t tcp_port;
    const gpio_num_t *reserved_pins;
    size_t reserved_pin_count;
} ninep_config_t;

esp_err_t ninep_start(const ninep_config_t *config);

#ifdef __cplusplus
}
#endif
