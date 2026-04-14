#include "lcd.h"

#include <string.h>

#include "config.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_caps.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/tjpgd.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/rom/tjpgd.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/tjpgd.h"
#endif

static const char *LCD_UTIL_TAG = "lcd";

#if ESP32_LCD && CONFIG_IDF_TARGET_ESP32S3

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4

#define EXAMPLE_LCD_H_RES               (800)
#define EXAMPLE_LCD_V_RES               (480)
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (16 * 1000 * 1000)
#define EXAMPLE_LCD_BIT_PER_PIXEL       (16)
#define EXAMPLE_RGB_BIT_PER_PIXEL       (16)
#define EXAMPLE_RGB_DATA_WIDTH          (16)
#define EXAMPLE_RGB_BOUNCE_BUFFER_SIZE  (EXAMPLE_LCD_H_RES * CONFIG_EXAMPLE_LCD_RGB_BOUNCE_BUFFER_HEIGHT)
#define EXAMPLE_LCD_IO_RGB_DISP         (-1)             // -1 if not used
#define EXAMPLE_LCD_IO_RGB_VSYNC        (GPIO_NUM_3)
#define EXAMPLE_LCD_IO_RGB_HSYNC        (GPIO_NUM_46)
#define EXAMPLE_LCD_IO_RGB_DE           (GPIO_NUM_5)
#define EXAMPLE_LCD_IO_RGB_PCLK         (GPIO_NUM_7)
#define EXAMPLE_LCD_IO_RGB_DATA0        (GPIO_NUM_14)
#define EXAMPLE_LCD_IO_RGB_DATA1        (GPIO_NUM_38)
#define EXAMPLE_LCD_IO_RGB_DATA2        (GPIO_NUM_18)
#define EXAMPLE_LCD_IO_RGB_DATA3        (GPIO_NUM_17)
#define EXAMPLE_LCD_IO_RGB_DATA4        (GPIO_NUM_10)
#define EXAMPLE_LCD_IO_RGB_DATA5        (GPIO_NUM_39)
#define EXAMPLE_LCD_IO_RGB_DATA6        (GPIO_NUM_0)
#define EXAMPLE_LCD_IO_RGB_DATA7        (GPIO_NUM_45)
#define EXAMPLE_LCD_IO_RGB_DATA8        (GPIO_NUM_48)
#define EXAMPLE_LCD_IO_RGB_DATA9        (GPIO_NUM_47)
#define EXAMPLE_LCD_IO_RGB_DATA10       (GPIO_NUM_21)
#define EXAMPLE_LCD_IO_RGB_DATA11       (GPIO_NUM_1)
#define EXAMPLE_LCD_IO_RGB_DATA12       (GPIO_NUM_2)
#define EXAMPLE_LCD_IO_RGB_DATA13       (GPIO_NUM_42)
#define EXAMPLE_LCD_IO_RGB_DATA14       (GPIO_NUM_41)
#define EXAMPLE_LCD_IO_RGB_DATA15       (GPIO_NUM_40)

#define LCD_H          800
#define LCD_W          480

esp_lcd_panel_handle_t panel;
static bool s_lcd_ready;

void lcd_fill_rect(uint16_t color) {
    uint16_t line[LCD_H];

    for (int i = 0; i < LCD_H; i++) {
        line[i] = color;
    }

    for (int y = 0; y < LCD_W; y++) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_H, y + 1, line);
    }
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    // Configure I2C parameters
    i2c_param_config(i2c_master_port, &i2c_conf);

    // Install I2C driver
    return i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0);
}

// GPIO initialization
void gpio_init(void)
{
    // Zero-initialize the config structure
    gpio_config_t io_conf = {};
    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // Set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    gpio_config(&io_conf);
}

esp_err_t waveshare_rgb_lcd_bl_on()
{
    //Configure CH422G to output mode 
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    //Pull the backlight pin high to light the screen backlight 
    write_buf = 0x1E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

// Typical starting timing for many 800x480 RGB panels.
// Tune later if your board doc lists different porch/polarity.
esp_err_t lcd_init(void) {
    esp_log_level_set("lcd_panel.rgb", ESP_LOG_DEBUG);
    esp_log_level_set("esp_lcd", ESP_LOG_DEBUG);

    esp_lcd_rgb_panel_config_t cfg = {
        .data_width = 16,                 // RGB565 bus
        .bits_per_pixel = 16,             // you already generate RGB565
        .num_fbs = 0,                     // start with 1 FB (PSRAM OK)
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .timings =  {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, // Pixel clock frequency
            .h_res = EXAMPLE_LCD_H_RES, // Horizontal resolution
            .v_res = EXAMPLE_LCD_V_RES, // Vertical resolution
            .hsync_pulse_width = 4, // Horizontal sync pulse width
            .hsync_back_porch = 8, // Horizontal back porch
            .hsync_front_porch = 8, // Horizontal front porch
            .vsync_pulse_width = 4, // Vertical sync pulse width
            .vsync_back_porch = 8, // Vertical back porch
            .vsync_front_porch = 8, // Vertical front porch
            .flags = {
                .pclk_active_neg = 1, // Active low pixel clock
            },
        },
        .bounce_buffer_size_px = LCD_H * 10,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC, // GPIO number for horizontal sync
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC, // GPIO number for vertical sync
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE, // GPIO number for data enable
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK, // GPIO number for pixel clock
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP, // GPIO number for display
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .flags = {
            .fb_in_psram = 1, // Use PSRAM for framebuffer
        },
        .clk_src = LCD_CLK_SRC_DEFAULT, // leave default
    };

    // If your glass uses a controller that needs init commands (e.g. ST7701/GC9xxx),
    // add the matching vendor driver before/after panel init (see esp_lcd_st7701).
    // Otherwise raw RGB panels just work with the timings above.
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    //ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    i2c_master_init(); // Initialize the I2C master
    gpio_init(); // Initialize GPIO pins
    waveshare_rgb_lcd_bl_on();
    lcd_fill_rect(0x000f);
    s_lcd_ready = true;
    return ESP_OK;
}

void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *array) {
    esp_lcd_panel_draw_bitmap(panel, x, y, w + x, y + h + 1, array);

}

#elif ESP32_LCD && CONFIG_IDF_TARGET_ESP32C6

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"

esp_lcd_panel_handle_t panel;
static bool s_lcd_ready;

#define PIN_LCD_MOSI   6
#define PIN_LCD_SCLK   7
#define PIN_LCD_CS     14
#define PIN_LCD_DC     15
#define PIN_LCD_RST    21
#define PIN_LCD_BL     22

#define LCD_H          320
#define LCD_W          172
#define LCD_X_GAP      34   // adjust if image is shifted; try 0 or 34
#define LCD_Y_GAP      0

static const char *TAG = "st7789";

void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *array) {
    for (int dy = y; dy < (h + y); dy++) {
        esp_lcd_panel_draw_bitmap(panel, x, dy, x + w, dy + 1, array);
        array += w;
    }
}

void lcd_fill_rect(uint16_t color)
{
    // Fill a test rect (white)
    uint16_t line[LCD_W];
    for (int i = 0; i < LCD_W; ++i) line[i] = color;
    for (int y = 0; y < LCD_H; ++y) {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_W, y+1, line));
    }
}

esp_err_t lcd_init(void)
{
    // Backlight pin
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LCD_BL, 0); // off until init

    // SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCLK,
        .max_transfer_sz = LCD_W * LCD_H * 2, // safe upper bound
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Panel IO over SPI
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,   // try 40 MHz
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    // ST7789 panel
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,   // swap if colors look wrong
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, LCD_X_GAP, LCD_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    gpio_set_level(PIN_LCD_BL, 1); // backlight on

    // Fill a test rect (white)
    lcd_fill_rect(0x000f);
    s_lcd_ready = true;
    ESP_LOGI(TAG, "Display init done");
    return ESP_OK;
}

#else

static bool s_lcd_ready;

esp_err_t lcd_init(void)
{
    s_lcd_ready = false;
    return ESP_ERR_NOT_SUPPORTED;
}

void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels)
{
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    (void)pixels;
}

void lcd_fill_rect(uint16_t color)
{
    (void)color;
}

#endif

bool lcd_is_ready(void)
{
    return s_lcd_ready;
}

uint16_t lcd_width(void)
{
#if ESP32_LCD && CONFIG_IDF_TARGET_ESP32S3
    return LCD_H;
#elif ESP32_LCD && CONFIG_IDF_TARGET_ESP32C6
    return LCD_W;
#else
    return 0;
#endif
}

uint16_t lcd_height(void)
{
#if ESP32_LCD && CONFIG_IDF_TARGET_ESP32S3
    return LCD_W;
#elif ESP32_LCD && CONFIG_IDF_TARGET_ESP32C6
    return LCD_H;
#else
    return 0;
#endif
}

size_t lcd_framebuffer_bytes(void)
{
    return (size_t)lcd_width() * (size_t)lcd_height() * sizeof(uint16_t);
}

esp_err_t lcd_write_framebuffer(const void *data, size_t len)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, "lcd", "framebuffer is NULL");

    if (!lcd_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len != lcd_framebuffer_bytes()) {
        return ESP_ERR_INVALID_SIZE;
    }

    lcd_blit(0, 0, lcd_width(), lcd_height(), data);
    return ESP_OK;
}

static inline uint16_t lcd_rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((uint16_t)(r & 0xF8) << 8) |
                      ((uint16_t)(g & 0xFC) << 3) |
                      ((uint16_t)(b & 0xF8) >> 3));
}

esp_err_t lcd_draw_rgb32(uint16_t origin_x, uint16_t origin_y, size_t byte_offset, const void *data, size_t len)
{
    const uint8_t *src = (const uint8_t *)data;
    uint16_t panel_w = lcd_width();
    uint16_t panel_h = lcd_height();
    size_t pixel_index;
    size_t pixels_left;

    ESP_RETURN_ON_FALSE(data != NULL || len == 0, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "rgb buffer is NULL");
    ESP_RETURN_ON_FALSE((byte_offset % 4U) == 0, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "rgb write offset must be pixel aligned");
    ESP_RETURN_ON_FALSE((len % 4U) == 0, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "rgb write length must be pixel aligned");
    ESP_RETURN_ON_FALSE(lcd_is_ready(), ESP_ERR_INVALID_STATE, LCD_UTIL_TAG, "lcd is not ready");

    if (len == 0 || panel_w == 0 || panel_h == 0 || origin_x >= panel_w || origin_y >= panel_h) {
        return ESP_OK;
    }

    pixel_index = byte_offset / 4U;
    pixels_left = len / 4U;

    while (pixels_left > 0) {
        size_t src_x = pixel_index % panel_w;
        size_t src_y = pixel_index / panel_w;
        size_t row_pixels = pixels_left;
        size_t draw_x = (size_t)origin_x + src_x;
        size_t visible = 0;

        if (src_y + origin_y >= panel_h) {
            break;
        }

        if (row_pixels > panel_w - src_x) {
            row_pixels = panel_w - src_x;
        }

        if (draw_x < panel_w) {
            uint16_t linebuf[panel_w];

            visible = row_pixels;
            if (visible > panel_w - draw_x) {
                visible = panel_w - draw_x;
            }

            for (size_t i = 0; i < visible; ++i) {
                const uint8_t *pixel = src + (i * 4U);
                linebuf[i] = lcd_rgb888_to_rgb565(pixel[0], pixel[1], pixel[2]);
            }

            lcd_blit((uint16_t)draw_x, (uint16_t)(origin_y + src_y), (uint16_t)visible, 1, linebuf);
        }

        src += row_pixels * 4U;
        pixels_left -= row_pixels;
        pixel_index += row_pixels;
    }

    return ESP_OK;
}

#if ESP_ROM_HAS_JPEG_DECODE
typedef struct {
    const uint8_t *jpeg_data;
    size_t jpeg_len;
    size_t jpeg_off;
    uint16_t *image;
    uint16_t image_width;
    uint16_t image_height;
    uint16_t origin_x;
    uint16_t origin_y;
    esp_err_t err;
} lcd_jpeg_ctx_t;

static UINT lcd_jpeg_input(JDEC *decoder, BYTE *buf, UINT len)
{
    lcd_jpeg_ctx_t *ctx = (lcd_jpeg_ctx_t *)decoder->device;
    size_t remaining = ctx->jpeg_off < ctx->jpeg_len ? ctx->jpeg_len - ctx->jpeg_off : 0;
    size_t amount = len;

    if (amount > remaining) {
        amount = remaining;
    }
    if (buf != NULL && amount > 0) {
        memcpy(buf, ctx->jpeg_data + ctx->jpeg_off, amount);
    }
    ctx->jpeg_off += amount;
    return (UINT)amount;
}

static UINT lcd_jpeg_output_to_buffer(JDEC *decoder, void *bitmap, JRECT *rect)
{
    lcd_jpeg_ctx_t *ctx = (lcd_jpeg_ctx_t *)decoder->device;
    const uint8_t *src = (const uint8_t *)bitmap;
    uint16_t block_w = (uint16_t)(rect->right - rect->left + 1);
    uint16_t block_h = (uint16_t)(rect->bottom - rect->top + 1);

    if (ctx->image == NULL) {
        ctx->err = ESP_ERR_INVALID_STATE;
        return 0;
    }

    for (uint16_t y = 0; y < block_h; ++y) {
        uint16_t *dst = ctx->image + ((size_t)(rect->top + y) * ctx->image_width) + rect->left;
        for (uint16_t x = 0; x < block_w; ++x) {
            dst[x] = lcd_rgb888_to_rgb565(src[0], src[1], src[2]);
            src += 3;
        }
    }

    return 1;
}

static UINT lcd_jpeg_output_direct(JDEC *decoder, void *bitmap, JRECT *rect)
{
    lcd_jpeg_ctx_t *ctx = (lcd_jpeg_ctx_t *)decoder->device;
    const uint8_t *src = (const uint8_t *)bitmap;
    uint16_t panel_w = lcd_width();
    uint16_t panel_h = lcd_height();
    uint16_t block_w = (uint16_t)(rect->right - rect->left + 1);
    uint16_t block_h = (uint16_t)(rect->bottom - rect->top + 1);
    uint16_t draw_y = (uint16_t)(ctx->origin_y + rect->top);

    for (uint16_t y = 0; y < block_h; ++y) {
        uint16_t linebuf[block_w];
        size_t draw_x = (size_t)ctx->origin_x + rect->left;
        size_t visible = block_w;

        if (draw_y + y >= panel_h) {
            break;
        }

        if (draw_x < panel_w) {
            if (visible > panel_w - draw_x) {
                visible = panel_w - draw_x;
            }
            for (size_t x = 0; x < visible; ++x) {
                linebuf[x] = lcd_rgb888_to_rgb565(src[0], src[1], src[2]);
                src += 3;
            }
            if (visible > 0) {
                lcd_blit((uint16_t)draw_x, (uint16_t)(draw_y + y), (uint16_t)visible, 1, linebuf);
            }
            src += (block_w - visible) * 3U;
        } else {
            src += (size_t)block_w * 3U;
        }
    }

    return 1;
}

static esp_err_t lcd_scale_rgb565_to_panel(uint16_t dst_x, uint16_t dst_y, uint16_t dst_w, uint16_t dst_h,
                                           const uint16_t *src, uint16_t src_w, uint16_t src_h)
{
    ESP_RETURN_ON_FALSE(src != NULL, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "scaled image is NULL");

    if (dst_w == 0 || dst_h == 0 || src_w == 0 || src_h == 0) {
        return ESP_OK;
    }

    for (uint16_t y = 0; y < dst_h; ++y) {
        uint16_t linebuf[dst_w];
        uint32_t src_y = ((uint32_t)y * src_h) / dst_h;
        const uint16_t *src_row = src + ((size_t)src_y * src_w);

        for (uint16_t x = 0; x < dst_w; ++x) {
            uint32_t src_x = ((uint32_t)x * src_w) / dst_w;
            linebuf[x] = src_row[src_x];
        }

        lcd_blit(dst_x, dst_y + y, dst_w, 1, linebuf);
    }

    return ESP_OK;
}
#endif

esp_err_t lcd_draw_jpeg(uint16_t origin_x, uint16_t origin_y, lcd_scale_mode_t scale, const void *data, size_t len)
{
#if ESP_ROM_HAS_JPEG_DECODE
    uint16_t panel_w = lcd_width();
    uint16_t panel_h = lcd_height();
    uint16_t viewport_w;
    uint16_t viewport_h;
    lcd_jpeg_ctx_t ctx = {
        .jpeg_data = (const uint8_t *)data,
        .jpeg_len = len,
        .origin_x = origin_x,
        .origin_y = origin_y,
        .err = ESP_OK,
    };
    JDEC decoder = { 0 };
    JRESULT result;
    size_t workbuf_size = 16 * 1024;
    void *workbuf = NULL;
    uint16_t dst_x;
    uint16_t dst_y;
    uint16_t dst_w;
    uint16_t dst_h;

    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "jpeg buffer is NULL");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "jpeg buffer is empty");
    ESP_RETURN_ON_FALSE(lcd_is_ready(), ESP_ERR_INVALID_STATE, LCD_UTIL_TAG, "lcd is not ready");
    ESP_RETURN_ON_FALSE(origin_x < panel_w && origin_y < panel_h, ESP_ERR_INVALID_ARG, LCD_UTIL_TAG, "jpeg origin is outside lcd");

    viewport_w = panel_w - origin_x;
    viewport_h = panel_h - origin_y;
    workbuf = heap_caps_malloc(workbuf_size, MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(workbuf != NULL, ESP_ERR_NO_MEM, LCD_UTIL_TAG, "failed to allocate jpeg work buffer");

    result = jd_prepare(&decoder, lcd_jpeg_input, workbuf, workbuf_size, &ctx);
    if (result != JDR_OK) {
        heap_caps_free(workbuf);
        ESP_LOGW(LCD_UTIL_TAG, "jpeg prepare failed: %d", result);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ctx.image_width = (uint16_t)decoder.width;
    ctx.image_height = (uint16_t)decoder.height;

    if (scale == LCD_SCALE_NONE) {
        result = jd_decomp(&decoder, lcd_jpeg_output_direct, 0);
        heap_caps_free(workbuf);
        if (result != JDR_OK || ctx.err != ESP_OK) {
            ESP_LOGW(LCD_UTIL_TAG, "jpeg decode failed: %d", result);
            return ctx.err != ESP_OK ? ctx.err : ESP_ERR_INVALID_RESPONSE;
        }
        return ESP_OK;
    }

    {
        size_t image_bytes = (size_t)ctx.image_width * (size_t)ctx.image_height * sizeof(uint16_t);

#if CONFIG_IDF_TARGET_ESP32S3
        ctx.image = heap_caps_malloc(image_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        if (ctx.image == NULL) {
            ctx.image = heap_caps_malloc(image_bytes, MALLOC_CAP_8BIT);
        }
#else
        ctx.image = heap_caps_malloc(image_bytes, MALLOC_CAP_8BIT);
#endif

        if (ctx.image == NULL) {
            heap_caps_free(workbuf);
            return ESP_ERR_NO_MEM;
        }
    }

    result = jd_decomp(&decoder, lcd_jpeg_output_to_buffer, 0);
    heap_caps_free(workbuf);
    if (result != JDR_OK || ctx.err != ESP_OK) {
        heap_caps_free(ctx.image);
        ESP_LOGW(LCD_UTIL_TAG, "jpeg decode failed: %d", result);
        return ctx.err != ESP_OK ? ctx.err : ESP_ERR_INVALID_RESPONSE;
    }

    dst_x = origin_x;
    dst_y = origin_y;
    dst_w = viewport_w;
    dst_h = viewport_h;

    if (scale == LCD_SCALE_ASPECT) {
        uint32_t scaled_w = viewport_w;
        uint32_t scaled_h = viewport_h;

        if ((uint32_t)ctx.image_width * viewport_h > (uint32_t)viewport_w * ctx.image_height) {
            scaled_h = ((uint32_t)ctx.image_height * viewport_w) / ctx.image_width;
        } else {
            scaled_w = ((uint32_t)ctx.image_width * viewport_h) / ctx.image_height;
        }

        if (scaled_w == 0) {
            scaled_w = 1;
        }
        if (scaled_h == 0) {
            scaled_h = 1;
        }

        dst_w = (uint16_t)scaled_w;
        dst_h = (uint16_t)scaled_h;
        dst_x = origin_x + (uint16_t)((viewport_w - dst_w) / 2U);
        dst_y = origin_y + (uint16_t)((viewport_h - dst_h) / 2U);
    }

    if (scale == LCD_SCALE_FULL || scale == LCD_SCALE_ASPECT) {
        esp_err_t err = lcd_scale_rgb565_to_panel(dst_x, dst_y, dst_w, dst_h, ctx.image, ctx.image_width, ctx.image_height);
        heap_caps_free(ctx.image);
        return err;
    }

    heap_caps_free(ctx.image);
    return ESP_OK;
#else
    (void)origin_x;
    (void)origin_y;
    (void)scale;
    (void)data;
    (void)len;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
