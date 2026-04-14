#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

#define ESP32_AUDIO 1
#define ESP32_LCD   1
#define ESP32_HTTP  0

#define AUDIO_I2S_BCLK_GPIO     GPIO_NUM_13
#define AUDIO_I2S_WS_GPIO       GPIO_NUM_14
#define AUDIO_I2S_DOUT_GPIO     GPIO_NUM_12
#define AUDIO_I2S_MCLK_GPIO     GPIO_NUM_NC
#define AUDIO_MUTE_GPIO         GPIO_NUM_27
#define AUDIO_MUTE_ACTIVE_HIGH  true
