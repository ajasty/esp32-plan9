#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LCD_SCALE_NONE = 0,
    LCD_SCALE_ASPECT,
    LCD_SCALE_FULL,
} lcd_scale_mode_t;

esp_err_t lcd_init(void);
bool lcd_is_ready(void);
uint16_t lcd_width(void);
uint16_t lcd_height(void);
size_t lcd_framebuffer_bytes(void);
esp_err_t lcd_write_framebuffer(const void *data, size_t len);
esp_err_t lcd_draw_rgb32(uint16_t origin_x, uint16_t origin_y, size_t byte_offset, const void *data, size_t len);
esp_err_t lcd_draw_jpeg(uint16_t origin_x, uint16_t origin_y, lcd_scale_mode_t scale, const void *data, size_t len);
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels);
void lcd_fill_rect(uint16_t color);

#ifdef __cplusplus
}
#endif
