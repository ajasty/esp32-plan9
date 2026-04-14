#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/rmt_types.h"

typedef struct {
    uint16_t addr;
    uint16_t cmd;
    bool repeat;
    bool addr_check_ok;
    bool cmd_check_ok;
} ir_nec_frame_t;

// Initialize RMT RX/TX for IR.
// rx_gpio: GPIO for IR receiver output
// tx_gpio: GPIO for IR LED drive (set < 0 to disable TX)
// active_high: true if receiver output is high during IR carrier
esp_err_t ir_init(int rx_gpio, int tx_gpio, bool active_high);

// Start a FreeRTOS task that receives and decodes IR, printing results.
esp_err_t ir_start_rx_task(void);

// Transmit a NEC frame (address + command).
esp_err_t ir_send_nec(uint16_t addr, uint16_t cmd);

// Transmit raw RMT symbols as-is.
esp_err_t ir_send_raw(const rmt_symbol_word_t *symbols, size_t num_symbols);
