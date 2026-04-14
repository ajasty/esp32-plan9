#include "ir_rmt.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define IR_RMT_RESOLUTION_HZ     1000000
#define IR_RMT_RX_BUF_SYMBOLS    128
#define IR_RMT_TX_BUF_SYMBOLS    64

#define IR_NEC_LEADER_PULSE_US   9000
#define IR_NEC_LEADER_SPACE_US   4500
#define IR_NEC_REPEAT_SPACE_US   2250
#define IR_NEC_BIT_PULSE_US      562
#define IR_NEC_BIT_0_SPACE_US    562
#define IR_NEC_BIT_1_SPACE_US    1687
#define IR_NEC_STOP_PULSE_US     562

#define IR_TOLERANCE_PCT         25

static const char *TAG = "ir_rmt";

static rmt_channel_handle_t s_rx_chan;
static rmt_channel_handle_t s_tx_chan;
static rmt_encoder_handle_t s_copy_encoder;
static QueueHandle_t s_rx_queue;
static bool s_active_high;

static void ir_log_pwm_bits(const uint8_t *bits, size_t count);

static inline bool ir_match_us(uint32_t actual, uint32_t target)
{
    uint32_t tol = (target * IR_TOLERANCE_PCT) / 100;
    return (actual >= (target - tol)) && (actual <= (target + tol));
}

static void ir_dump_symbols(const rmt_symbol_word_t *symbols, size_t num_symbols)
{
    for (size_t i = 0; i < num_symbols; ++i) {
        const rmt_symbol_word_t *s = &symbols[i];
        ESP_LOGI(TAG, "sym[%u]: %u/%u  %u/%u",
                 (unsigned)i,
                 (unsigned)s->level0, (unsigned)s->duration0,
                 (unsigned)s->level1, (unsigned)s->duration1);
    }
}

static bool ir_nec_is_leader(const rmt_symbol_word_t *s, uint32_t pulse_level, bool *is_repeat)
{
    if (s->level0 != pulse_level || s->level1 == pulse_level) {
        return false;
    }
    if (!ir_match_us(s->duration0, IR_NEC_LEADER_PULSE_US)) {
        return false;
    }
    if (ir_match_us(s->duration1, IR_NEC_REPEAT_SPACE_US)) {
        *is_repeat = true;
        return true;
    }
    if (ir_match_us(s->duration1, IR_NEC_LEADER_SPACE_US)) {
        *is_repeat = false;
        return true;
    }
    return false;
}

static bool ir_decode_nec_from_index(const rmt_symbol_word_t *symbols,
                                     size_t num_symbols,
                                     size_t start_idx,
                                     uint32_t pulse_level,
                                     ir_nec_frame_t *out)
{
    if (start_idx >= num_symbols || (num_symbols - start_idx) < 2) {
        return false;
    }

    bool is_repeat = false;
    const rmt_symbol_word_t *leader = &symbols[start_idx];
    if (!ir_nec_is_leader(leader, pulse_level, &is_repeat)) {
        return false;
    }
    if (is_repeat) {
        out->repeat = true;
        out->addr = 0;
        out->cmd = 0;
        out->addr_check_ok = true;
        out->cmd_check_ok = true;
        return true;
    }
    if ((num_symbols - start_idx) < (1 + 32)) {
        return false;
    }

    uint32_t data = 0;
    for (int i = 0; i < 32; ++i) {
        const rmt_symbol_word_t *s = &symbols[start_idx + 1 + i];
        if (s->level0 != pulse_level || s->level1 == pulse_level) {
            return false;
        }
        if (!ir_match_us(s->duration0, IR_NEC_BIT_PULSE_US)) {
            return false;
        }
        if (ir_match_us(s->duration1, IR_NEC_BIT_0_SPACE_US)) {
            // bit = 0
        } else if (ir_match_us(s->duration1, IR_NEC_BIT_1_SPACE_US)) {
            data |= (1UL << i);
        } else {
            return false;
        }
    }

    const uint8_t addr = data & 0xFF;
    const uint8_t addr_inv = (data >> 8) & 0xFF;
    const uint8_t cmd = (data >> 16) & 0xFF;
    const uint8_t cmd_inv = (data >> 24) & 0xFF;

    out->repeat = false;
    out->addr_check_ok = ((uint8_t)(addr ^ addr_inv) == 0xFF);
    out->cmd_check_ok = ((uint8_t)(cmd ^ cmd_inv) == 0xFF);

    if (out->addr_check_ok) {
        out->addr = addr;
    } else {
        // Extended NEC: 16-bit address
        out->addr = data & 0xFFFF;
    }
    out->cmd = cmd;

    return true;
}

static bool ir_decode_nec(const rmt_symbol_word_t *symbols,
                          size_t num_symbols,
                          bool active_high,
                          ir_nec_frame_t *out)
{
    if (num_symbols < 2) {
        return false;
    }

    const uint32_t pulse_level = active_high ? 1 : 0;
    for (size_t i = 0; i + 1 < num_symbols; ++i) {
        if (ir_decode_nec_from_index(symbols, num_symbols, i, pulse_level, out)) {
            return true;
        }
    }
    return false;
}

static bool ir_decode_nec_auto_polarity(const rmt_symbol_word_t *symbols,
                                        size_t num_symbols,
                                        ir_nec_frame_t *out)
{
    if (ir_decode_nec(symbols, num_symbols, true, out)) {
        return true;
    }
    return ir_decode_nec(symbols, num_symbols, false, out);
}

static bool ir_decode_rc5_from_frame(const rmt_symbol_word_t *symbols,
                                     size_t num_symbols,
                                     size_t start_idx,
                                     size_t end_idx,
                                     uint8_t *out_addr,
                                     uint8_t *out_cmd,
                                     uint8_t *out_toggle)
{
    const size_t max_bits = 32;
    uint8_t bits[max_bits];
    size_t bit_count = 0;

    for (size_t i = start_idx; i < end_idx && bit_count < max_bits; ++i) {
        const rmt_symbol_word_t *s = &symbols[i];
        if (s->level0 == s->level1) {
            return false;
        }
        uint8_t bit;
        if (s->level0 == 1 && s->level1 == 0) {
            bit = 1;
        } else if (s->level0 == 0 && s->level1 == 1) {
            bit = 0;
        } else {
            return false;
        }
        bits[bit_count++] = bit;
    }

    for (size_t i = 0; i + 14 <= bit_count; ++i) {
        if (bits[i] == 1 && bits[i + 1] == 1) {
            *out_toggle = bits[i + 2];
            uint8_t addr = 0;
            uint8_t cmd = 0;
            for (int b = 0; b < 5; ++b) {
                addr = (addr << 1) | bits[i + 3 + b];
            }
            for (int b = 0; b < 6; ++b) {
                cmd = (cmd << 1) | bits[i + 8 + b];
            }
            *out_addr = addr;
            *out_cmd = cmd;
            return true;
        }
    }

    return false;
}

static bool ir_decode_rc5(const rmt_symbol_word_t *symbols,
                          size_t num_symbols,
                          uint8_t *out_addr,
                          uint8_t *out_cmd,
                          uint8_t *out_toggle)
{
    const uint32_t max_bit_us = 3000;
    size_t frame_start = 0;
    size_t i = 0;

    while (i < num_symbols) {
        const rmt_symbol_word_t *s = &symbols[i];
        uint32_t total = s->duration0 + s->duration1;
        if (total > max_bit_us || total < 500) {
            if (i > frame_start) {
                if (ir_decode_rc5_from_frame(symbols, num_symbols, frame_start, i,
                                             out_addr, out_cmd, out_toggle)) {
                    return true;
                }
                if (ir_decode_rc5_from_frame(symbols, num_symbols, frame_start, i,
                                             out_addr, out_cmd, out_toggle)) {
                    return true;
                }
            }
            frame_start = i + 1;
        }
        ++i;
    }

    if (frame_start < num_symbols) {
        if (ir_decode_rc5_from_frame(symbols, num_symbols, frame_start, num_symbols,
                                     out_addr, out_cmd, out_toggle)) {
            return true;
        }
        if (ir_decode_rc5_from_frame(symbols, num_symbols, frame_start, num_symbols,
                                     out_addr, out_cmd, out_toggle)) {
            return true;
        }
    }

    return false;
}

static bool ir_decode_rc6(const rmt_symbol_word_t *symbols,
                          size_t num_symbols,
                          uint8_t *out_mode,
                          uint8_t *out_toggle,
                          uint8_t *out_addr,
                          uint8_t *out_cmd)
{
    const size_t max_halves = 128;
    uint8_t half_levels[max_halves];
    size_t half_count = 0;

    for (size_t i = 0; i < num_symbols && half_count + 1 < max_halves; ++i) {
        const rmt_symbol_word_t *s = &symbols[i];
        half_levels[half_count++] = s->level0 ? 1 : 0;
        half_levels[half_count++] = s->level1 ? 1 : 0;
    }

    size_t idx = 0;
    if (half_count < 8) {
        return false;
    }

    uint8_t decode_pair_bit = 0;
    #define IR_RC6_DECODE_PAIR(bit_ptr) \
        do { \
            if (idx + 1 >= half_count) { \
                return false; \
            } \
            uint8_t a = half_levels[idx++]; \
            uint8_t b = half_levels[idx++]; \
            if (a == b) { \
                return false; \
            } \
            *(bit_ptr) = (a == 1 && b == 0) ? 1 : 0; \
        } while (0)

    uint8_t start = 0;
    IR_RC6_DECODE_PAIR(&start);
    if (start != 1) {
        return false;
    }

    uint8_t mode = 0;
    for (int i = 0; i < 3; ++i) {
        IR_RC6_DECODE_PAIR(&decode_pair_bit);
        mode = (mode << 1) | decode_pair_bit;
    }

    if (idx + 3 >= half_count) {
        return false;
    }
    uint8_t t0 = half_levels[idx];
    uint8_t t1 = half_levels[idx + 1];
    uint8_t t2 = half_levels[idx + 2];
    uint8_t t3 = half_levels[idx + 3];
    if (t0 != t1 || t2 != t3 || t0 == t2) {
        return false;
    }
    uint8_t toggle = (t0 == 1 && t2 == 0) ? 1 : 0;
    idx += 4;

    uint8_t addr = 0;
    for (int i = 0; i < 8; ++i) {
        IR_RC6_DECODE_PAIR(&decode_pair_bit);
        addr = (addr << 1) | decode_pair_bit;
    }

    uint8_t cmd = 0;
    for (int i = 0; i < 8; ++i) {
        IR_RC6_DECODE_PAIR(&decode_pair_bit);
        cmd = (cmd << 1) | decode_pair_bit;
    }

    #undef IR_RC6_DECODE_PAIR

    *out_mode = mode;
    *out_toggle = toggle;
    *out_addr = addr;
    *out_cmd = cmd;
    return true;
}

static bool ir_extract_pwm_frame(const rmt_symbol_word_t *symbols,
                                 size_t start_idx,
                                 size_t end_idx,
                                 bool active_high,
                                 uint8_t *bits,
                                 size_t max_bits,
                                 size_t *out_count,
                                 uint32_t *out_threshold,
                                 uint32_t *out_short,
                                 uint32_t *out_long)
{
    uint32_t min_high = UINT32_MAX;
    uint32_t max_high = 0;
    size_t bit_count = 0;
    uint16_t highs[128];
    size_t cap = sizeof(highs) / sizeof(highs[0]);

    for (size_t i = start_idx; i < end_idx && bit_count < max_bits; ++i) {
        const rmt_symbol_word_t *s = &symbols[i];
        if (s->level0 == s->level1) {
            continue;
        }

        uint32_t high_us = 0;
        if (active_high) {
            high_us = (s->level0 == 1) ? s->duration0 : s->duration1;
        } else {
            high_us = (s->level0 == 0) ? s->duration0 : s->duration1;
        }
        uint32_t total = s->duration0 + s->duration1;

        if (total > 3000 || high_us == 0) {
            continue;
        }
        if (total < 200 || high_us < 100) {
            // Filter tiny glitches
            continue;
        }

        if (bit_count >= cap) {
            break;
        }

        if (high_us < min_high) {
            min_high = high_us;
        }
        if (high_us > max_high) {
            max_high = high_us;
        }
        highs[bit_count++] = (uint16_t)high_us;
    }

    if (bit_count < 8 || min_high == UINT32_MAX) {
        return false;
    }
    if (max_high < min_high + 100) {
        return false;
    }

    uint32_t threshold = (min_high + max_high) / 2;
    for (size_t i = 0; i < bit_count; ++i) {
        bits[i] = (highs[i] >= threshold) ? 1 : 0;
    }

    *out_count = bit_count;
    *out_threshold = threshold;
    *out_short = min_high;
    *out_long = max_high;
    return true;
}

static bool ir_decode_pwm_simple(const rmt_symbol_word_t *symbols,
                                 size_t num_symbols,
                                 bool active_high)
{
    uint8_t bits[128];
    size_t bit_count = 0;
    uint32_t threshold = 0;
    uint32_t short_hi = 0;
    uint32_t long_hi = 0;

    if (!ir_extract_pwm_frame(symbols, 0, num_symbols,
                              active_high, bits, sizeof(bits),
                              &bit_count, &threshold, &short_hi, &long_hi)) {
        return false;
    }

    ESP_LOGI(TAG, "PWM decode: short=%u long=%u thr=%u",
             (unsigned)short_hi, (unsigned)long_hi, (unsigned)threshold);
    ir_log_pwm_bits(bits, bit_count);
    return true;
}

static void ir_log_pwm_bits(const uint8_t *bits, size_t count)
{
    const size_t max_bits = 128;
    char bit_str[max_bits + 1];
    size_t n = (count > max_bits) ? max_bits : count;

    for (size_t i = 0; i < n; ++i) {
        bit_str[i] = bits[i] ? '1' : '0';
    }
    bit_str[n] = '\0';
    ESP_LOGI(TAG, "PWM bits[%u]: %s%s",
             (unsigned)count, bit_str, (count > max_bits) ? "..." : "");

    size_t byte_count = count / 8;
    if (byte_count == 0) {
        return;
    }
    char hex_str[3 * 16 + 1];
    size_t hex_pos = 0;
    size_t max_bytes = 16;
    size_t bytes_to_print = (byte_count > max_bytes) ? max_bytes : byte_count;

    for (size_t b = 0; b < bytes_to_print; ++b) {
        uint8_t val = 0;
        for (int i = 0; i < 8; ++i) {
            val = (val << 1) | bits[b * 8 + i];
        }
        int written = snprintf(&hex_str[hex_pos], sizeof(hex_str) - hex_pos, "%02X ", val);
        if (written < 0) {
            break;
        }
        hex_pos += (size_t)written;
        if (hex_pos >= sizeof(hex_str) - 1) {
            break;
        }
    }
    if (hex_pos > 0 && hex_str[hex_pos - 1] == ' ') {
        hex_str[hex_pos - 1] = '\0';
    } else {
        hex_str[hex_pos] = '\0';
    }
    ESP_LOGI(TAG, "PWM bytes: %s%s", hex_str, (byte_count > max_bytes) ? " ..." : "");
}

static bool ir_rx_done_cb(rmt_channel_handle_t channel,
                          const rmt_rx_done_event_data_t *edata,
                          void *user_data)
{
    QueueHandle_t queue = (QueueHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void ir_rx_task(void *arg)
{
    rmt_symbol_word_t *rx_symbols = NULL;
    const size_t rx_size = IR_RMT_RX_BUF_SYMBOLS * sizeof(rmt_symbol_word_t);
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 15000000,
    };

    rx_symbols = heap_caps_malloc(rx_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rx_symbols) {
        ESP_LOGE(TAG, "RX symbol buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        rmt_rx_done_event_data_t rx_data = {0};
        esp_err_t err = rmt_receive(s_rx_chan, rx_symbols, rx_size, &rx_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "rmt_receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (xQueueReceive(s_rx_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            if (rx_data.num_symbols == 0) {
                continue;
            }

            ESP_LOGI(TAG, "RX symbols: %u", (unsigned)rx_data.num_symbols);
            ir_dump_symbols(rx_data.received_symbols, rx_data.num_symbols);

            ir_nec_frame_t frame = {0};
            if (ir_decode_nec_auto_polarity(rx_data.received_symbols, rx_data.num_symbols, &frame)) {
                if (frame.repeat) {
                    ESP_LOGI(TAG, "NEC repeat");
                } else {
                    ESP_LOGI(TAG, "NEC addr=0x%04x cmd=0x%02x addr_ok=%d cmd_ok=%d",
                             frame.addr, frame.cmd,
                             frame.addr_check_ok, frame.cmd_check_ok);
                }
            } else {
                uint8_t rc5_addr = 0;
                uint8_t rc5_cmd = 0;
                uint8_t rc5_toggle = 0;
                if (ir_decode_rc5(rx_data.received_symbols, rx_data.num_symbols,
                                  &rc5_addr, &rc5_cmd, &rc5_toggle)) {
                    ESP_LOGI(TAG, "RC5 addr=0x%02x cmd=0x%02x toggle=%u",
                             rc5_addr, rc5_cmd, rc5_toggle);
                } else {
                    uint8_t rc6_mode = 0;
                    uint8_t rc6_toggle = 0;
                    uint8_t rc6_addr = 0;
                    uint8_t rc6_cmd = 0;
                    if (ir_decode_rc6(rx_data.received_symbols, rx_data.num_symbols,
                                      &rc6_mode, &rc6_toggle, &rc6_addr, &rc6_cmd)) {
                        ESP_LOGI(TAG, "RC6 mode=%u addr=0x%02x cmd=0x%02x toggle=%u",
                                 rc6_mode, rc6_addr, rc6_cmd, rc6_toggle);
                    } else {
                        if (!ir_decode_pwm_simple(rx_data.received_symbols,
                                                  rx_data.num_symbols,
                                                  s_active_high)) {
                            ESP_LOGW(TAG, "NEC/RC5/RC6/PWM decode failed");
                        }
                    }
                }
            }
        }
    }
}

static void ir_set_symbol(rmt_symbol_word_t *sym,
                          uint16_t dur0,
                          uint16_t dur1,
                          bool level0,
                          bool level1)
{
    sym->duration0 = dur0;
    sym->level0 = level0;
    sym->duration1 = dur1;
    sym->level1 = level1;
}

esp_err_t ir_init(int rx_gpio, int tx_gpio, bool active_high)
{
    esp_err_t ret;
    s_active_high = active_high;

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RMT_RESOLUTION_HZ,
        .mem_block_symbols = IR_RMT_RX_BUF_SYMBOLS,
        .intr_priority = 0,
    };

    ret = rmt_new_rx_channel(&rx_cfg, &s_rx_chan);
    ESP_RETURN_ON_ERROR(ret, TAG, "rmt_new_rx_channel");

    s_rx_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    if (!s_rx_queue) {
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ir_rx_done_cb,
    };
    ret = rmt_rx_register_event_callbacks(s_rx_chan, &cbs, s_rx_queue);
    ESP_RETURN_ON_ERROR(ret, TAG, "rmt_rx_register_event_callbacks");

    ret = rmt_enable(s_rx_chan);
    ESP_RETURN_ON_ERROR(ret, TAG, "rmt_enable rx");

    if (tx_gpio >= 0) {
        rmt_tx_channel_config_t tx_cfg = {
            .gpio_num = tx_gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = IR_RMT_RESOLUTION_HZ,
            .mem_block_symbols = IR_RMT_TX_BUF_SYMBOLS,
            .trans_queue_depth = 4,
        };
        ret = rmt_new_tx_channel(&tx_cfg, &s_tx_chan);
        ESP_RETURN_ON_ERROR(ret, TAG, "rmt_new_tx_channel");

        rmt_carrier_config_t carrier_cfg = {
            .frequency_hz = 38000,
            .duty_cycle = 0.25f,
            .flags = {
                .polarity_active_low = 0,
                .always_on = 0,
            },
        };
        ret = rmt_apply_carrier(s_tx_chan, &carrier_cfg);
        ESP_RETURN_ON_ERROR(ret, TAG, "rmt_apply_carrier");

        rmt_copy_encoder_config_t copy_cfg = {
        };
        ret = rmt_new_copy_encoder(&copy_cfg, &s_copy_encoder);
        ESP_RETURN_ON_ERROR(ret, TAG, "rmt_new_copy_encoder");

        ret = rmt_enable(s_tx_chan);
        ESP_RETURN_ON_ERROR(ret, TAG, "rmt_enable tx");
    }

    return ESP_OK;
}

esp_err_t ir_start_rx_task(void)
{
    if (!s_rx_chan || !s_rx_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskCreate(ir_rx_task, "ir_rx", 4096, NULL, 10, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ir_send_nec(uint16_t addr, uint16_t cmd)
{
    if (!s_tx_chan || !s_copy_encoder) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_symbol_word_t symbols[1 + 32 + 1] = {0};
    size_t idx = 0;
    const bool pulse_level = true;
    const bool space_level = false;

    ir_set_symbol(&symbols[idx++],
                  IR_NEC_LEADER_PULSE_US,
                  IR_NEC_LEADER_SPACE_US,
                  pulse_level, space_level);

    uint32_t data = 0;
    data |= (uint32_t)(addr & 0xFF);
    data |= (uint32_t)((uint8_t)~addr) << 8;
    data |= (uint32_t)(cmd & 0xFF) << 16;
    data |= (uint32_t)((uint8_t)~cmd) << 24;

    for (int i = 0; i < 32; ++i) {
        const bool bit = (data >> i) & 1U;
        const uint16_t space = bit ? IR_NEC_BIT_1_SPACE_US : IR_NEC_BIT_0_SPACE_US;
        ir_set_symbol(&symbols[idx++],
                      IR_NEC_BIT_PULSE_US,
                      space,
                      pulse_level, space_level);
    }

    ir_set_symbol(&symbols[idx++],
                  IR_NEC_STOP_PULSE_US,
                  0,
                  pulse_level, space_level);

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_chan, s_copy_encoder, symbols,
                                     idx * sizeof(rmt_symbol_word_t), &tx_cfg),
                        TAG, "rmt_transmit");
    return rmt_tx_wait_all_done(s_tx_chan, portMAX_DELAY);
}

esp_err_t ir_send_raw(const rmt_symbol_word_t *symbols, size_t num_symbols)
{
    if (!s_tx_chan || !s_copy_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!symbols || num_symbols == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_chan, s_copy_encoder, symbols,
                                     num_symbols * sizeof(rmt_symbol_word_t), &tx_cfg),
                        TAG, "rmt_transmit");
    return rmt_tx_wait_all_done(s_tx_chan, portMAX_DELAY);
}
