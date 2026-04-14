#include "esp_check.h"
#include "esp_log.h"

#include "audio.h"
#include "config.h"
#include "lcd.h"
#include "ninep.h"
#include "webserver.h"

#include "esp_wifi.h"

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "st7789";

#if ESP32_AUDIO
static audio_pins_t default_audio_pins(void)
{
    return (audio_pins_t) {
        .bclk = AUDIO_I2S_BCLK_GPIO,
        .ws = AUDIO_I2S_WS_GPIO,
        .dout = AUDIO_I2S_DOUT_GPIO,
        .mclk = AUDIO_I2S_MCLK_GPIO,
        .mute = AUDIO_MUTE_GPIO,
        .mute_active_high = AUDIO_MUTE_ACTIVE_HIGH,
    };
}

static void maybe_start_audio(const audio_pins_t *audio_pins)
{
    if (audio_pins->bclk == GPIO_NUM_NC || audio_pins->ws == GPIO_NUM_NC || audio_pins->dout == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "Audio disabled: set AUDIO_I2S_*_GPIO pins in config.h to enable /pcm writes");
        return;
    }

    ESP_ERROR_CHECK(audio_start(audio_pins));
}
#endif

static void init_wifi(void)
{
  esp_netif_create_default_wifi_sta(); // or _ap()
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t sta = { .sta = { .ssid = "h7j8a1sI", .password = "DenverOil" } };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main(void)
{
  gpio_num_t reserved_pins[5];
  size_t reserved_pin_count = 0;
  ninep_config_t ninep_cfg = {
      .tcp_port = 564,
      .reserved_pins = reserved_pins,
      .reserved_pin_count = 0,
  };

#if ESP32_AUDIO
  audio_pins_t audio_pins = default_audio_pins();
  reserved_pins[reserved_pin_count++] = audio_pins.bclk;
  reserved_pins[reserved_pin_count++] = audio_pins.ws;
  reserved_pins[reserved_pin_count++] = audio_pins.dout;
  reserved_pins[reserved_pin_count++] = audio_pins.mclk;
  reserved_pins[reserved_pin_count++] = audio_pins.mute;
#endif
  ninep_cfg.reserved_pin_count = reserved_pin_count;

  heap_caps_print_heap_info(MALLOC_CAP_DMA);
  heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);

#if ESP32_LCD
  esp_err_t lcd_err = lcd_init();
  if (lcd_err != ESP_OK && lcd_err != ESP_ERR_NOT_SUPPORTED) {
      ESP_LOGW(TAG, "LCD init failed: %s", esp_err_to_name(lcd_err));
  }
#endif

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  init_wifi();

#if ESP32_HTTP
  start_webserver();
#endif
#if ESP32_AUDIO
  maybe_start_audio(&audio_pins);
#endif
  ESP_ERROR_CHECK(ninep_start(&ninep_cfg));
}
