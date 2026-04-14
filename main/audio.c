#include "audio.h"

#include <string.h>

#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define AUDIO_DEFAULT_SAMPLE_RATE_HZ 44100U
#define AUDIO_MIN_SAMPLE_RATE_HZ      8000U
#define AUDIO_MAX_SAMPLE_RATE_HZ      192000U
#define AUDIO_BITS_PER_SAMPLE         16U
#define AUDIO_CHANNELS                2U
#define AUDIO_FRAME_BYTES             ((AUDIO_BITS_PER_SAMPLE / 8U) * AUDIO_CHANNELS)
#define AUDIO_STARTUP_BUFFER_MS       200U
#define AUDIO_RING_BUFFER_BYTES       (64 * 1024U)
#define AUDIO_DMA_DESC_NUM            8
#define AUDIO_DMA_FRAME_NUM           480
#define AUDIO_IO_CHUNK_BYTES          4096U
#define AUDIO_SERVER_TASK_STACK       4096
#define AUDIO_SERVER_TASK_PRIORITY    5
#define AUDIO_I2S_WRITE_TIMEOUT_MS    1000
#define AUDIO_DRAIN_PAD_MS            10U
#define AUDIO_WRITE_WAIT_TICKS        pdMS_TO_TICKS(10)
#define AUDIO_IDLE_WAIT_TICKS         pdMS_TO_TICKS(5)

typedef struct {
    audio_pins_t pins;
    i2s_chan_handle_t tx_chan;
    TaskHandle_t output_task;
    SemaphoreHandle_t lock;
    uint8_t *ring;
    size_t head;
    size_t tail;
    size_t used;
    uint32_t sample_rate_hz;
    bool started;
    bool i2s_enabled;
} audio_state_t;

static const char *TAG = "audio";

static audio_state_t s_audio;
static uint8_t s_tx_chunk[AUDIO_IO_CHUNK_BYTES];
static uint8_t s_silence_chunk[AUDIO_IO_CHUNK_BYTES];

static inline size_t min_size(size_t a, size_t b)
{
    return a < b ? a : b;
}

static inline bool audio_mute_pin_is_used(void)
{
    return s_audio.pins.mute != GPIO_NUM_NC;
}

static inline void audio_set_muted(bool muted)
{
    if (!audio_mute_pin_is_used()) {
        return;
    }

    gpio_set_level(s_audio.pins.mute, muted ? s_audio.pins.mute_active_high : !s_audio.pins.mute_active_high);
}

static inline void ring_reset_locked(void)
{
    s_audio.head = 0;
    s_audio.tail = 0;
    s_audio.used = 0;
}

static inline size_t ring_free_locked(void)
{
    return AUDIO_RING_BUFFER_BYTES - s_audio.used;
}

static inline size_t ring_playable_locked(void)
{
    return s_audio.used - (s_audio.used % AUDIO_FRAME_BYTES);
}

static inline size_t audio_startup_buffer_bytes_locked(void)
{
    size_t bytes = ((size_t)s_audio.sample_rate_hz * AUDIO_FRAME_BYTES * AUDIO_STARTUP_BUFFER_MS) / 1000U;

    return bytes > 0 ? bytes : AUDIO_FRAME_BYTES;
}

static inline uint32_t audio_buffered_ms_locked(void)
{
    uint64_t denom = (uint64_t)s_audio.sample_rate_hz * AUDIO_FRAME_BYTES;

    if (denom == 0) {
        return 0;
    }

    return (uint32_t)(((uint64_t)s_audio.used * 1000ULL) / denom);
}

static void ring_write_locked(const uint8_t *src, size_t len)
{
    while (len > 0) {
        size_t chunk = min_size(len, AUDIO_RING_BUFFER_BYTES - s_audio.head);
        memcpy(&s_audio.ring[s_audio.head], src, chunk);
        s_audio.head = (s_audio.head + chunk) % AUDIO_RING_BUFFER_BYTES;
        s_audio.used += chunk;
        src += chunk;
        len -= chunk;
    }
}

static void ring_peek_locked(uint8_t *dst, size_t len)
{
    size_t offset = s_audio.tail;

    while (len > 0) {
        size_t chunk = min_size(len, AUDIO_RING_BUFFER_BYTES - offset);
        memcpy(dst, &s_audio.ring[offset], chunk);
        offset = (offset + chunk) % AUDIO_RING_BUFFER_BYTES;
        dst += chunk;
        len -= chunk;
    }
}

static void ring_drop_locked(size_t len)
{
    s_audio.tail = (s_audio.tail + len) % AUDIO_RING_BUFFER_BYTES;
    s_audio.used -= len;
}

static void audio_cleanup_startup(void)
{
    if (s_audio.tx_chan != NULL) {
        i2s_del_channel(s_audio.tx_chan);
    }

    if (s_audio.lock != NULL) {
        vSemaphoreDelete(s_audio.lock);
    }

    if (s_audio.ring != NULL) {
        heap_caps_free(s_audio.ring);
    }

    memset(&s_audio, 0, sizeof(s_audio));
}

static esp_err_t audio_stop_output_locked(bool pad_with_silence)
{
    if (!s_audio.i2s_enabled) {
        audio_set_muted(true);
        return ESP_OK;
    }

    if (pad_with_silence) {
        size_t bytes_written = 0;
        if (i2s_channel_write(s_audio.tx_chan, s_silence_chunk, sizeof(s_silence_chunk), &bytes_written,
                              AUDIO_I2S_WRITE_TIMEOUT_MS) == ESP_OK &&
            bytes_written > 0) {
            uint32_t frames_written = (uint32_t)((bytes_written + (AUDIO_FRAME_BYTES - 1U)) / AUDIO_FRAME_BYTES);
            uint32_t drain_ms = s_audio.sample_rate_hz > 0 ? (frames_written * 1000U) / s_audio.sample_rate_hz : 0;
            vTaskDelay(pdMS_TO_TICKS(drain_ms + AUDIO_DRAIN_PAD_MS));
        }
    }

    esp_err_t err = i2s_channel_disable(s_audio.tx_chan);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "i2s disable failed: %s", esp_err_to_name(err));
        return err;
    }

    s_audio.i2s_enabled = false;
    audio_set_muted(true);
    return ESP_OK;
}

static esp_err_t audio_start_output_locked(void)
{
    if (s_audio.i2s_enabled) {
        return ESP_OK;
    }

    size_t preload_size = min_size(ring_playable_locked(), sizeof(s_tx_chunk));
    preload_size -= preload_size % AUDIO_FRAME_BYTES;
    if (preload_size > 0) {
        size_t bytes_loaded = 0;

        ring_peek_locked(s_tx_chunk, preload_size);
        ESP_RETURN_ON_ERROR(i2s_channel_preload_data(s_audio.tx_chan, s_tx_chunk, preload_size, &bytes_loaded), TAG,
                            "failed to preload I2S DMA");
        if (bytes_loaded > 0) {
            ring_drop_locked(bytes_loaded);
        }
    }

    audio_set_muted(false);
    esp_err_t err = i2s_channel_enable(s_audio.tx_chan);
    if (err != ESP_OK) {
        audio_set_muted(true);
        return err;
    }

    s_audio.i2s_enabled = true;
    ESP_LOGI(TAG, "I2S output enabled at %u Hz", s_audio.sample_rate_hz);
    return ESP_OK;
}

static esp_err_t audio_reconfig_rate_locked(uint32_t sample_rate_hz)
{
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    bool was_enabled = s_audio.i2s_enabled;

    if (was_enabled) {
        ESP_RETURN_ON_ERROR(audio_stop_output_locked(false), TAG, "failed to stop audio before rate change");
    }

    ESP_RETURN_ON_ERROR(i2s_channel_reconfig_std_clock(s_audio.tx_chan, &clk_cfg), TAG, "failed to set sample rate");
    s_audio.sample_rate_hz = sample_rate_hz;
    ring_reset_locked();
    ESP_LOGI(TAG, "Audio sample rate set to %u Hz", sample_rate_hz);
    return ESP_OK;
}

static void audio_output_task(void *arg)
{
    (void)arg;

    while (1) {
        bool did_work = false;

        if (xSemaphoreTake(s_audio.lock, portMAX_DELAY) == pdTRUE) {
            size_t playable = ring_playable_locked();

            if (!s_audio.i2s_enabled) {
                if (playable >= audio_startup_buffer_bytes_locked()) {
                    if (audio_start_output_locked() == ESP_OK) {
                        did_work = true;
                    }
                }
            } else if (playable == 0) {
                audio_stop_output_locked(true);
                did_work = true;
            } else {
                size_t chunk_size = min_size(playable, sizeof(s_tx_chunk));
                chunk_size -= chunk_size % AUDIO_FRAME_BYTES;

                if (chunk_size > 0) {
                    size_t bytes_written = 0;

                    ring_peek_locked(s_tx_chunk, chunk_size);
                    esp_err_t err = i2s_channel_write(s_audio.tx_chan, s_tx_chunk, chunk_size, &bytes_written,
                                                      AUDIO_I2S_WRITE_TIMEOUT_MS);
                    if (bytes_written > 0) {
                        ring_drop_locked(bytes_written);
                    }
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "I2S write failed, muting output until more PCM arrives");
                        audio_stop_output_locked(false);
                    }
                    did_work = true;
                }
            }

            xSemaphoreGive(s_audio.lock);
        }

        if (!did_work) {
            vTaskDelay(AUDIO_IDLE_WAIT_TICKS);
        }
    }
}

esp_err_t audio_start(const audio_pins_t *pins)
{
    ESP_RETURN_ON_FALSE(pins != NULL, ESP_ERR_INVALID_ARG, TAG, "pins is NULL");
    ESP_RETURN_ON_FALSE(pins->bclk != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "bclk pin is required");
    ESP_RETURN_ON_FALSE(pins->ws != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "ws pin is required");
    ESP_RETURN_ON_FALSE(pins->dout != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "dout pin is required");
    ESP_RETURN_ON_FALSE(!s_audio.started, ESP_ERR_INVALID_STATE, TAG, "audio already started");

    memset(&s_audio, 0, sizeof(s_audio));
    s_audio.pins = *pins;
    s_audio.sample_rate_hz = AUDIO_DEFAULT_SAMPLE_RATE_HZ;
    s_audio.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_audio.lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create audio mutex");

    s_audio.ring = heap_caps_malloc(AUDIO_RING_BUFFER_BYTES, MALLOC_CAP_8BIT);
    if (s_audio.ring == NULL) {
        audio_cleanup_startup();
        return ESP_ERR_NO_MEM;
    }

    if (audio_mute_pin_is_used()) {
        gpio_config_t mute_io = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << s_audio.pins.mute,
        };
        esp_err_t err = gpio_config(&mute_io);
        if (err != ESP_OK) {
            audio_cleanup_startup();
            return err;
        }
    }
    audio_set_muted(true);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = AUDIO_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = AUDIO_DMA_FRAME_NUM;
    chan_cfg.auto_clear_after_cb = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_audio.tx_chan, NULL);
    if (err != ESP_OK) {
        audio_cleanup_startup();
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_audio.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = pins->mclk,
            .bclk = pins->bclk,
            .ws = pins->ws,
            .dout = pins->dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    err = i2s_channel_init_std_mode(s_audio.tx_chan, &std_cfg);
    if (err != ESP_OK) {
        audio_cleanup_startup();
        return err;
    }

    BaseType_t task_ok = xTaskCreate(audio_output_task, "audio_out", AUDIO_SERVER_TASK_STACK, NULL,
                                     AUDIO_SERVER_TASK_PRIORITY, &s_audio.output_task);
    if (task_ok != pdPASS) {
        audio_cleanup_startup();
        return ESP_ERR_NO_MEM;
    }

    s_audio.started = true;
    ESP_LOGI(TAG, "Audio ready: %u Hz, 16-bit stereo", s_audio.sample_rate_hz);
    return ESP_OK;
}

esp_err_t audio_write(const void *data, size_t len, TickType_t timeout_ticks, size_t *written)
{
    const uint8_t *src = data;
    TickType_t start_tick;
    size_t total = 0;

    if (written != NULL) {
        *written = 0;
    }

    if (len == 0) {
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(src != NULL, ESP_ERR_INVALID_ARG, TAG, "data is NULL");
    ESP_RETURN_ON_FALSE(s_audio.started, ESP_ERR_INVALID_STATE, TAG, "audio is not ready");

    start_tick = xTaskGetTickCount();

    while (total < len) {
        if (xSemaphoreTake(s_audio.lock, portMAX_DELAY) == pdTRUE) {
            size_t free_bytes = ring_free_locked();
            if (free_bytes > 0) {
                size_t chunk = min_size(free_bytes, len - total);
                ring_write_locked(src + total, chunk);
                total += chunk;
            }
            xSemaphoreGive(s_audio.lock);
        }

        if (total == len) {
            break;
        }

        if (timeout_ticks == 0 || (xTaskGetTickCount() - start_tick) >= timeout_ticks) {
            break;
        }

        vTaskDelay(AUDIO_WRITE_WAIT_TICKS);
    }

    if (written != NULL) {
        *written = total;
    }

    return total == len ? ESP_OK : ESP_ERR_TIMEOUT;
}

uint32_t audio_get_sample_rate_hz(void)
{
    uint32_t sample_rate_hz = AUDIO_DEFAULT_SAMPLE_RATE_HZ;

    if (!s_audio.started || s_audio.lock == NULL) {
        return sample_rate_hz;
    }

    if (xSemaphoreTake(s_audio.lock, portMAX_DELAY) == pdTRUE) {
        sample_rate_hz = s_audio.sample_rate_hz;
        xSemaphoreGive(s_audio.lock);
    }

    return sample_rate_hz;
}

esp_err_t audio_set_sample_rate_hz(uint32_t sample_rate_hz)
{
    esp_err_t err = ESP_OK;

    ESP_RETURN_ON_FALSE(s_audio.started, ESP_ERR_INVALID_STATE, TAG, "audio is not ready");
    ESP_RETURN_ON_FALSE(sample_rate_hz >= AUDIO_MIN_SAMPLE_RATE_HZ && sample_rate_hz <= AUDIO_MAX_SAMPLE_RATE_HZ,
                        ESP_ERR_INVALID_ARG, TAG, "invalid sample rate");

    if (xSemaphoreTake(s_audio.lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (sample_rate_hz != s_audio.sample_rate_hz) {
        err = audio_reconfig_rate_locked(sample_rate_hz);
    }

    xSemaphoreGive(s_audio.lock);
    return err;
}

uint32_t audio_get_buffered_ms(void)
{
    uint32_t buffered_ms = 0;

    if (!s_audio.started || s_audio.lock == NULL) {
        return 0;
    }

    if (xSemaphoreTake(s_audio.lock, portMAX_DELAY) == pdTRUE) {
        buffered_ms = audio_buffered_ms_locked();
        xSemaphoreGive(s_audio.lock);
    }

    return buffered_ms;
}
