#include "ninep.h"

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_log_write.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

#include "audio.h"
#include "config.h"
#include "lcd.h"

#define NINEP_DEFAULT_PORT          564
#define NINEP_DEFAULT_MSIZE         8192U
#define NINEP_MAX_MSIZE             8192U
#define NINEP_MIN_MSIZE             256U
#define NINEP_LISTEN_BACKLOG        1
#define NINEP_SERVER_TASK_STACK     8192
#define NINEP_SERVER_TASK_PRIORITY  5
#define NINEP_NOFID                 0xFFFFFFFFU
#define NINEP_NOTAG                 0xFFFFU
#define NINEP_CONSOLE_BYTES         8192
#define NINEP_UID                   "esp"
#define NINEP_GID                   "esp"
#define NINEP_MUID                  "esp"
#define NINEP_VERSION               "9P2000"
#define NINEP_GPIO_PWM_FREQ_HZ      5000U
#define NINEP_GPIO_PWM_TIMER        LEDC_TIMER_0
#define NINEP_GPIO_PWM_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define NINEP_GPIO_PWM_RESOLUTION   LEDC_TIMER_10_BIT
#define NINEP_GPIO_PWM_MAX_DUTY     ((1U << 10) - 1U)

#define P9_TVERSION 100
#define P9_RVERSION 101
#define P9_TAUTH    102
#define P9_RAUTH    103
#define P9_TATTACH  104
#define P9_RATTACH  105
#define P9_RERROR   107
#define P9_TFLUSH   108
#define P9_RFLUSH   109
#define P9_TWALK    110
#define P9_RWALK    111
#define P9_TOPEN    112
#define P9_ROPEN    113
#define P9_TCREATE  114
#define P9_RCREATE  115
#define P9_TREAD    116
#define P9_RREAD    117
#define P9_TWRITE   118
#define P9_RWRITE   119
#define P9_TCLUNK   120
#define P9_RCLUNK   121
#define P9_TREMOVE  122
#define P9_RREMOVE  123
#define P9_TSTAT    124
#define P9_RSTAT    125
#define P9_TWSTAT   126
#define P9_RWSTAT   127

#define P9_OREAD    0x00
#define P9_OWRITE   0x01
#define P9_ORDWR    0x02
#define P9_OEXEC    0x03

#define P9_QTDIR    0x80
#define P9_DMDIR    0x80000000U

typedef enum {
    NINEP_NODE_ROOT,
    NINEP_NODE_CONSOLE,
    NINEP_NODE_GPIO_DIR,
    NINEP_NODE_GPIO_MODE_DIR,
    NINEP_NODE_GPIO_MODE_PIN,
    NINEP_NODE_GPIO_FLAGS_DIR,
    NINEP_NODE_GPIO_FLAGS_PIN,
    NINEP_NODE_GPIO_PWM_DIR,
    NINEP_NODE_GPIO_PWM_PIN,
    NINEP_NODE_GPIO_ADC_DIR,
    NINEP_NODE_GPIO_ADC_PIN,
    NINEP_NODE_GPIO_PIN,
    NINEP_NODE_PCM_DIR,
    NINEP_NODE_PCM_DAC,
    NINEP_NODE_PCM_RATE,
    NINEP_NODE_PCM_BUFFER,
    NINEP_NODE_LCD_DIR,
    NINEP_NODE_LCD_FRAMEBUFFER,
    NINEP_NODE_LCD_RGB,
    NINEP_NODE_LCD_JPEG,
    NINEP_NODE_LCD_SCALE,
    NINEP_NODE_LCD_X,
    NINEP_NODE_LCD_Y,
} ninep_node_kind_t;

typedef enum {
    NINEP_GPIO_MODE_DIG = 0,
    NINEP_GPIO_MODE_PWM,
    NINEP_GPIO_MODE_ADC,
    NINEP_GPIO_MODE_RSVD,
} ninep_gpio_mode_t;

enum {
    NINEP_GPIO_FLAG_INPUT      = 1U << 0,
    NINEP_GPIO_FLAG_OUTPUT     = 1U << 1,
    NINEP_GPIO_FLAG_OPEN_DRAIN = 1U << 2,
    NINEP_GPIO_FLAG_PULL_UP    = 1U << 3,
    NINEP_GPIO_FLAG_PULL_DOWN  = 1U << 4,
};

typedef struct {
    ninep_node_kind_t kind;
    int pin;
} ninep_node_t;

typedef struct ninep_fid {
    uint32_t fid;
    ninep_node_t node;
    bool opened;
    uint8_t open_mode;
    uint64_t console_open_seq;
    uint8_t *lcd_buffer;
    size_t lcd_fill;
    size_t lcd_capacity;
    struct ninep_fid *next;
} ninep_fid_t;

typedef struct {
    const uint8_t *buf;
    size_t len;
    size_t off;
    bool ok;
} ninep_reader_t;

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t off;
    bool ok;
} ninep_writer_t;

typedef struct {
    uint8_t type;
    uint32_t version;
    uint64_t path;
} ninep_qid_t;

typedef struct {
    int sock;
    uint32_t msize;
    uint8_t *rx_buf;
    uint8_t *tx_buf;
    ninep_fid_t *fids;
} ninep_client_t;

typedef struct {
    bool started;
    TaskHandle_t server_task;
    uint16_t tcp_port;
    uint64_t reserved_gpio_mask;
    vprintf_like_t prev_vprintf;
    uint8_t gpio_modes[GPIO_PIN_COUNT];
    uint8_t gpio_flags[GPIO_PIN_COUNT];
    bool pwm_timer_ready;
    int8_t pwm_channel_for_pin[GPIO_PIN_COUNT];
    int8_t pwm_pin_for_channel[LEDC_CHANNEL_MAX];
    uint32_t pwm_duty[GPIO_PIN_COUNT];
    adc_oneshot_unit_handle_t adc_units[2];
    uint32_t adc_channel_mask[2];
    uint8_t lcd_scale_mode;
    uint16_t lcd_cursor_x;
    uint16_t lcd_cursor_y;
} ninep_state_t;

static const char *TAG = "9p";

static ninep_state_t s_ninep;
static portMUX_TYPE s_console_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_console_ring[NINEP_CONSOLE_BYTES];
static uint64_t s_console_start_seq;
static uint64_t s_console_write_seq;
static unsigned s_console_open_count;

static inline size_t min_size(size_t a, size_t b)
{
    return a < b ? a : b;
}

static inline uint64_t ninep_gpio_bit(int pin)
{
    return 1ULL << pin;
}

static inline bool ninep_node_is_dir(const ninep_node_t *node)
{
    return node->kind == NINEP_NODE_ROOT || node->kind == NINEP_NODE_GPIO_DIR ||
           node->kind == NINEP_NODE_GPIO_MODE_DIR || node->kind == NINEP_NODE_GPIO_FLAGS_DIR ||
           node->kind == NINEP_NODE_GPIO_PWM_DIR || node->kind == NINEP_NODE_GPIO_ADC_DIR ||
           node->kind == NINEP_NODE_PCM_DIR || node->kind == NINEP_NODE_LCD_DIR;
}

static inline uint8_t ninep_base_open_mode(uint8_t mode)
{
    return mode & 0x03;
}

static const char *ninep_gpio_mode_name(ninep_gpio_mode_t mode)
{
    switch (mode) {
    case NINEP_GPIO_MODE_DIG:
        return "dig";
    case NINEP_GPIO_MODE_PWM:
        return "pwm";
    case NINEP_GPIO_MODE_ADC:
        return "adc";
    case NINEP_GPIO_MODE_RSVD:
        return "rsvd";
    default:
        return "dig";
    }
}

static const char *ninep_lcd_scale_name(lcd_scale_mode_t mode)
{
    switch (mode) {
    case LCD_SCALE_NONE:
        return "none";
    case LCD_SCALE_ASPECT:
        return "aspect";
    case LCD_SCALE_FULL:
        return "full";
    default:
        return "none";
    }
}

static ninep_gpio_mode_t ninep_gpio_mode_for_pin(int pin)
{
    return (ninep_gpio_mode_t)s_ninep.gpio_modes[pin];
}

static uint8_t ninep_gpio_flags_for_pin(int pin)
{
    return s_ninep.gpio_flags[pin];
}

static bool ninep_gpio_effective_output(int pin)
{
    ninep_gpio_mode_t mode = ninep_gpio_mode_for_pin(pin);
    uint8_t flags = ninep_gpio_flags_for_pin(pin);

    switch (mode) {
    case NINEP_GPIO_MODE_PWM:
        return true;
    case NINEP_GPIO_MODE_ADC:
    case NINEP_GPIO_MODE_RSVD:
        return false;
    case NINEP_GPIO_MODE_DIG:
    default:
        if ((flags & NINEP_GPIO_FLAG_OUTPUT) != 0) {
            return true;
        }
        if ((flags & NINEP_GPIO_FLAG_INPUT) != 0) {
            return false;
        }
        return GPIO_IS_VALID_OUTPUT_GPIO(pin);
    }
}

static bool ninep_gpio_effective_input(int pin)
{
    ninep_gpio_mode_t mode = ninep_gpio_mode_for_pin(pin);
    uint8_t flags = ninep_gpio_flags_for_pin(pin);

    switch (mode) {
    case NINEP_GPIO_MODE_PWM:
        return (flags & NINEP_GPIO_FLAG_INPUT) != 0;
    case NINEP_GPIO_MODE_ADC:
        return true;
    case NINEP_GPIO_MODE_RSVD:
        return false;
    case NINEP_GPIO_MODE_DIG:
    default:
        if ((flags & NINEP_GPIO_FLAG_INPUT) != 0) {
            return true;
        }
        if ((flags & NINEP_GPIO_FLAG_OUTPUT) != 0) {
            return false;
        }
        return true;
    }
}

static uint8_t ninep_gpio_effective_flags(int pin)
{
    uint8_t flags = ninep_gpio_flags_for_pin(pin);

    if (ninep_gpio_effective_input(pin)) {
        flags |= NINEP_GPIO_FLAG_INPUT;
    } else {
        flags &= ~NINEP_GPIO_FLAG_INPUT;
    }

    if (ninep_gpio_effective_output(pin)) {
        flags |= NINEP_GPIO_FLAG_OUTPUT;
    } else {
        flags &= ~NINEP_GPIO_FLAG_OUTPUT;
    }

    if (ninep_gpio_mode_for_pin(pin) == NINEP_GPIO_MODE_ADC) {
        flags &= ~NINEP_GPIO_FLAG_OPEN_DRAIN;
    }

    return flags;
}

static size_t ninep_format_gpio_flags(int pin, char *value, size_t value_len)
{
    uint8_t flags = ninep_gpio_effective_flags(pin);
    size_t used = 0;
    bool first = true;

    if (flags == 0) {
        snprintf(value, value_len, "none\n");
        return strlen(value);
    }

    value[0] = '\0';
    if ((flags & NINEP_GPIO_FLAG_INPUT) != 0) {
        used += snprintf(value + used, value_len - used, "%sinput", first ? "" : ",");
        first = false;
    }
    if ((flags & NINEP_GPIO_FLAG_OUTPUT) != 0) {
        used += snprintf(value + used, value_len - used, "%soutput", first ? "" : ",");
        first = false;
    }
    if ((flags & NINEP_GPIO_FLAG_OPEN_DRAIN) != 0) {
        used += snprintf(value + used, value_len - used, "%sopen_drain", first ? "" : ",");
        first = false;
    }
    if ((flags & NINEP_GPIO_FLAG_PULL_UP) != 0) {
        used += snprintf(value + used, value_len - used, "%spull_up", first ? "" : ",");
        first = false;
    }
    if ((flags & NINEP_GPIO_FLAG_PULL_DOWN) != 0) {
        used += snprintf(value + used, value_len - used, "%spull_down", first ? "" : ",");
        first = false;
    }
    snprintf(value + used, value_len - used, "\n");
    return strlen(value);
}

static bool ninep_node_can_read(const ninep_node_t *node)
{
    return node->kind == NINEP_NODE_ROOT || node->kind == NINEP_NODE_GPIO_DIR || node->kind == NINEP_NODE_GPIO_MODE_DIR ||
           node->kind == NINEP_NODE_GPIO_FLAGS_DIR || node->kind == NINEP_NODE_GPIO_PWM_DIR ||
           node->kind == NINEP_NODE_GPIO_ADC_DIR || node->kind == NINEP_NODE_GPIO_PIN ||
           node->kind == NINEP_NODE_GPIO_MODE_PIN || node->kind == NINEP_NODE_GPIO_FLAGS_PIN ||
           node->kind == NINEP_NODE_GPIO_PWM_PIN || node->kind == NINEP_NODE_GPIO_ADC_PIN ||
           node->kind == NINEP_NODE_PCM_DIR || node->kind == NINEP_NODE_PCM_RATE ||
           node->kind == NINEP_NODE_PCM_BUFFER || node->kind == NINEP_NODE_CONSOLE ||
           node->kind == NINEP_NODE_LCD_DIR || node->kind == NINEP_NODE_LCD_SCALE ||
           node->kind == NINEP_NODE_LCD_X || node->kind == NINEP_NODE_LCD_Y;
}

static bool ninep_node_can_write(const ninep_node_t *node)
{
    return node->kind == NINEP_NODE_GPIO_PIN || node->kind == NINEP_NODE_GPIO_MODE_PIN ||
           node->kind == NINEP_NODE_GPIO_FLAGS_PIN || node->kind == NINEP_NODE_GPIO_PWM_PIN ||
           node->kind == NINEP_NODE_PCM_DAC || node->kind == NINEP_NODE_PCM_RATE ||
           node->kind == NINEP_NODE_LCD_FRAMEBUFFER || node->kind == NINEP_NODE_LCD_RGB ||
           node->kind == NINEP_NODE_LCD_JPEG || node->kind == NINEP_NODE_LCD_SCALE ||
           node->kind == NINEP_NODE_LCD_X || node->kind == NINEP_NODE_LCD_Y;
}

static ninep_qid_t ninep_node_qid(const ninep_node_t *node)
{
    switch (node->kind) {
    case NINEP_NODE_ROOT:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 1 };
    case NINEP_NODE_CONSOLE:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 2 };
    case NINEP_NODE_GPIO_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 3 };
    case NINEP_NODE_GPIO_MODE_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 6 };
    case NINEP_NODE_GPIO_MODE_PIN:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x200 + (uint64_t)node->pin };
    case NINEP_NODE_GPIO_FLAGS_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 7 };
    case NINEP_NODE_GPIO_FLAGS_PIN:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x300 + (uint64_t)node->pin };
    case NINEP_NODE_GPIO_PWM_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 8 };
    case NINEP_NODE_GPIO_PWM_PIN:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x500 + (uint64_t)node->pin };
    case NINEP_NODE_GPIO_ADC_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 9 };
    case NINEP_NODE_GPIO_ADC_PIN:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x600 + (uint64_t)node->pin };
    case NINEP_NODE_GPIO_PIN:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x100 + (uint64_t)node->pin };
    case NINEP_NODE_PCM_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 4 };
    case NINEP_NODE_PCM_DAC:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x400 };
    case NINEP_NODE_PCM_RATE:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x401 };
    case NINEP_NODE_PCM_BUFFER:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x402 };
    case NINEP_NODE_LCD_DIR:
        return (ninep_qid_t) { .type = P9_QTDIR, .version = 0, .path = 5 };
    case NINEP_NODE_LCD_FRAMEBUFFER:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x700 };
    case NINEP_NODE_LCD_RGB:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x701 };
    case NINEP_NODE_LCD_JPEG:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x702 };
    case NINEP_NODE_LCD_SCALE:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x703 };
    case NINEP_NODE_LCD_X:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x704 };
    case NINEP_NODE_LCD_Y:
        return (ninep_qid_t) { .type = 0, .version = 0, .path = 0x705 };
    default:
        return (ninep_qid_t) { 0 };
    }
}

static void ninep_node_name(const ninep_node_t *node, char *buf, size_t buf_len)
{
    switch (node->kind) {
    case NINEP_NODE_ROOT:
        snprintf(buf, buf_len, "/");
        break;
    case NINEP_NODE_CONSOLE:
        snprintf(buf, buf_len, "console");
        break;
    case NINEP_NODE_GPIO_DIR:
        snprintf(buf, buf_len, "gpio");
        break;
    case NINEP_NODE_GPIO_MODE_DIR:
        snprintf(buf, buf_len, "mode");
        break;
    case NINEP_NODE_GPIO_MODE_PIN:
        snprintf(buf, buf_len, "%d", node->pin);
        break;
    case NINEP_NODE_GPIO_FLAGS_DIR:
        snprintf(buf, buf_len, "flags");
        break;
    case NINEP_NODE_GPIO_FLAGS_PIN:
        snprintf(buf, buf_len, "%d", node->pin);
        break;
    case NINEP_NODE_GPIO_PWM_DIR:
        snprintf(buf, buf_len, "pwm");
        break;
    case NINEP_NODE_GPIO_PWM_PIN:
        snprintf(buf, buf_len, "%d", node->pin);
        break;
    case NINEP_NODE_GPIO_ADC_DIR:
        snprintf(buf, buf_len, "adc");
        break;
    case NINEP_NODE_GPIO_ADC_PIN:
        snprintf(buf, buf_len, "%d", node->pin);
        break;
    case NINEP_NODE_GPIO_PIN:
        snprintf(buf, buf_len, "pin%d", node->pin);
        break;
    case NINEP_NODE_PCM_DIR:
        snprintf(buf, buf_len, "pcm");
        break;
    case NINEP_NODE_PCM_DAC:
        snprintf(buf, buf_len, "dac");
        break;
    case NINEP_NODE_PCM_RATE:
        snprintf(buf, buf_len, "rate");
        break;
    case NINEP_NODE_PCM_BUFFER:
        snprintf(buf, buf_len, "buffer");
        break;
    case NINEP_NODE_LCD_DIR:
        snprintf(buf, buf_len, "lcd");
        break;
    case NINEP_NODE_LCD_FRAMEBUFFER:
        snprintf(buf, buf_len, "framebuffer");
        break;
    case NINEP_NODE_LCD_RGB:
        snprintf(buf, buf_len, "rgb");
        break;
    case NINEP_NODE_LCD_JPEG:
        snprintf(buf, buf_len, "jpeg");
        break;
    case NINEP_NODE_LCD_SCALE:
        snprintf(buf, buf_len, "scale");
        break;
    case NINEP_NODE_LCD_X:
        snprintf(buf, buf_len, "x");
        break;
    case NINEP_NODE_LCD_Y:
        snprintf(buf, buf_len, "y");
        break;
    }
}

static uint32_t ninep_node_mode(const ninep_node_t *node)
{
    switch (node->kind) {
    case NINEP_NODE_ROOT:
    case NINEP_NODE_GPIO_DIR:
    case NINEP_NODE_GPIO_MODE_DIR:
    case NINEP_NODE_GPIO_FLAGS_DIR:
    case NINEP_NODE_GPIO_PWM_DIR:
    case NINEP_NODE_GPIO_ADC_DIR:
    case NINEP_NODE_PCM_DIR:
    case NINEP_NODE_LCD_DIR:
        return P9_DMDIR | 0555;
    case NINEP_NODE_CONSOLE:
        return 0444;
    case NINEP_NODE_GPIO_PIN:
    case NINEP_NODE_GPIO_MODE_PIN:
    case NINEP_NODE_GPIO_FLAGS_PIN:
    case NINEP_NODE_GPIO_PWM_PIN:
        return 0666;
    case NINEP_NODE_GPIO_ADC_PIN:
        return 0444;
    case NINEP_NODE_PCM_DAC:
    case NINEP_NODE_LCD_FRAMEBUFFER:
    case NINEP_NODE_LCD_RGB:
    case NINEP_NODE_LCD_JPEG:
        return 0222;
    case NINEP_NODE_PCM_RATE:
    case NINEP_NODE_LCD_SCALE:
    case NINEP_NODE_LCD_X:
    case NINEP_NODE_LCD_Y:
        return 0666;
    case NINEP_NODE_PCM_BUFFER:
        return 0444;
    default:
        return 0;
    }
}

static uint64_t ninep_node_length(const ninep_node_t *node)
{
    switch (node->kind) {
    case NINEP_NODE_GPIO_PIN:
        return 2;
    case NINEP_NODE_GPIO_MODE_PIN:
        return strlen(ninep_gpio_mode_name(ninep_gpio_mode_for_pin(node->pin))) + 1;
    case NINEP_NODE_GPIO_FLAGS_PIN: {
        char value[64];
        return ninep_format_gpio_flags(node->pin, value, sizeof(value));
    }
    case NINEP_NODE_GPIO_PWM_PIN: {
        char value[16];
        snprintf(value, sizeof(value), "%" PRIu32 "\n", s_ninep.pwm_duty[node->pin]);
        return strlen(value);
    }
    case NINEP_NODE_GPIO_ADC_PIN:
        return 12;
    case NINEP_NODE_PCM_RATE: {
        char value[16];
        snprintf(value, sizeof(value), "%" PRIu32 "\n", audio_get_sample_rate_hz());
        return strlen(value);
    }
    case NINEP_NODE_PCM_BUFFER: {
        char value[16];
        snprintf(value, sizeof(value), "%" PRIu32 "\n", audio_get_buffered_ms());
        return strlen(value);
    }
    case NINEP_NODE_LCD_FRAMEBUFFER:
        return lcd_framebuffer_bytes();
    case NINEP_NODE_LCD_SCALE:
        return strlen(ninep_lcd_scale_name((lcd_scale_mode_t)s_ninep.lcd_scale_mode)) + 1;
    case NINEP_NODE_LCD_X: {
        char value[16];
        snprintf(value, sizeof(value), "%" PRIu16 "\n", s_ninep.lcd_cursor_x);
        return strlen(value);
    }
    case NINEP_NODE_LCD_Y: {
        char value[16];
        snprintf(value, sizeof(value), "%" PRIu16 "\n", s_ninep.lcd_cursor_y);
        return strlen(value);
    }
    default:
        return 0;
    }
}

static bool ninep_gpio_is_exported(int pin)
{
    if (!GPIO_IS_VALID_GPIO(pin)) {
        return false;
    }
    if ((s_ninep.reserved_gpio_mask & ninep_gpio_bit(pin)) != 0) {
        return false;
    }
    return true;
}

static int ninep_gpio_count(void)
{
    int count = 0;

    for (int pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        if (ninep_gpio_is_exported(pin)) {
            ++count;
        }
    }

    return count;
}

static bool ninep_gpio_parse_name(const char *name, size_t len, int *pin_out)
{
    int pin = 0;

    if (len < 4 || strncmp(name, "pin", 3) != 0) {
        return false;
    }

    for (size_t i = 3; i < len; ++i) {
        if (!isdigit((unsigned char)name[i])) {
            return false;
        }
        pin = (pin * 10) + (name[i] - '0');
    }

    if (!ninep_gpio_is_exported(pin)) {
        return false;
    }

    *pin_out = pin;
    return true;
}

static bool ninep_gpio_parse_numeric_name(const char *name, size_t len, int *pin_out)
{
    int pin = 0;

    if (len == 0) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        if (!isdigit((unsigned char)name[i])) {
            return false;
        }
        pin = (pin * 10) + (name[i] - '0');
    }

    if (!ninep_gpio_is_exported(pin)) {
        return false;
    }

    *pin_out = pin;
    return true;
}

static bool ninep_walk_one(const ninep_node_t *from, const char *name, size_t len, ninep_node_t *to)
{
    if (len == 1 && name[0] == '.') {
        *to = *from;
        return true;
    }

    if (len == 2 && name[0] == '.' && name[1] == '.') {
        switch (from->kind) {
        case NINEP_NODE_ROOT:
            *to = *from;
            break;
        case NINEP_NODE_GPIO_DIR:
        case NINEP_NODE_CONSOLE:
        case NINEP_NODE_PCM_DIR:
        case NINEP_NODE_LCD_DIR:
            *to = (ninep_node_t) { .kind = NINEP_NODE_ROOT };
            break;
        case NINEP_NODE_GPIO_MODE_DIR:
        case NINEP_NODE_GPIO_FLAGS_DIR:
        case NINEP_NODE_GPIO_PWM_DIR:
        case NINEP_NODE_GPIO_ADC_DIR:
        case NINEP_NODE_GPIO_PIN:
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_DIR };
            break;
        case NINEP_NODE_GPIO_MODE_PIN:
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_MODE_DIR };
            break;
        case NINEP_NODE_GPIO_FLAGS_PIN:
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_FLAGS_DIR };
            break;
        case NINEP_NODE_GPIO_PWM_PIN:
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_PWM_DIR };
            break;
        case NINEP_NODE_GPIO_ADC_PIN:
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_ADC_DIR };
            break;
        case NINEP_NODE_PCM_DAC:
        case NINEP_NODE_PCM_RATE:
        case NINEP_NODE_PCM_BUFFER:
            *to = (ninep_node_t) { .kind = NINEP_NODE_PCM_DIR };
            break;
        case NINEP_NODE_LCD_FRAMEBUFFER:
        case NINEP_NODE_LCD_RGB:
        case NINEP_NODE_LCD_JPEG:
        case NINEP_NODE_LCD_SCALE:
        case NINEP_NODE_LCD_X:
        case NINEP_NODE_LCD_Y:
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_DIR };
            break;
        }
        return true;
    }

    if (from->kind == NINEP_NODE_ROOT) {
        if (len == 7 && strncmp(name, "console", 7) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_CONSOLE };
            return true;
        }
        if (len == 4 && strncmp(name, "gpio", 4) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_DIR };
            return true;
        }
        if (ESP32_AUDIO && len == 3 && strncmp(name, "pcm", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_PCM_DIR };
            return true;
        }
        if (ESP32_LCD && len == 3 && strncmp(name, "lcd", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_DIR };
            return true;
        }
        return false;
    }

    if (from->kind == NINEP_NODE_GPIO_DIR) {
        if (len == 4 && strncmp(name, "mode", 4) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_MODE_DIR };
            return true;
        }
        if (len == 5 && strncmp(name, "flags", 5) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_FLAGS_DIR };
            return true;
        }
        if (len == 3 && strncmp(name, "pwm", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_PWM_DIR };
            return true;
        }
        if (len == 3 && strncmp(name, "adc", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_ADC_DIR };
            return true;
        }

        int pin = -1;
        if (ninep_gpio_parse_name(name, len, &pin)) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_PIN, .pin = pin };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_GPIO_MODE_DIR) {
        int pin = -1;
        if (ninep_gpio_parse_numeric_name(name, len, &pin)) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_MODE_PIN, .pin = pin };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_GPIO_FLAGS_DIR) {
        int pin = -1;
        if (ninep_gpio_parse_numeric_name(name, len, &pin)) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_FLAGS_PIN, .pin = pin };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_GPIO_PWM_DIR) {
        int pin = -1;
        if (ninep_gpio_parse_numeric_name(name, len, &pin)) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_PWM_PIN, .pin = pin };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_GPIO_ADC_DIR) {
        int pin = -1;
        if (ninep_gpio_parse_numeric_name(name, len, &pin)) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_GPIO_ADC_PIN, .pin = pin };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_PCM_DIR) {
        if (len == 3 && strncmp(name, "dac", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_PCM_DAC };
            return true;
        }
        if (len == 4 && strncmp(name, "rate", 4) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_PCM_RATE };
            return true;
        }
        if (len == 6 && strncmp(name, "buffer", 6) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_PCM_BUFFER };
            return true;
        }
    }

    if (from->kind == NINEP_NODE_LCD_DIR) {
        if (len == 11 && strncmp(name, "framebuffer", 11) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_FRAMEBUFFER };
            return true;
        }
        if (len == 3 && strncmp(name, "rgb", 3) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_RGB };
            return true;
        }
        if (len == 4 && strncmp(name, "jpeg", 4) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_JPEG };
            return true;
        }
        if (len == 5 && strncmp(name, "scale", 5) == 0) {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_SCALE };
            return true;
        }
        if (len == 1 && name[0] == 'x') {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_X };
            return true;
        }
        if (len == 1 && name[0] == 'y') {
            *to = (ninep_node_t) { .kind = NINEP_NODE_LCD_Y };
            return true;
        }
    }

    return false;
}

static ninep_fid_t *ninep_find_fid(ninep_client_t *client, uint32_t fid)
{
    for (ninep_fid_t *curr = client->fids; curr != NULL; curr = curr->next) {
        if (curr->fid == fid) {
            return curr;
        }
    }
    return NULL;
}

static void ninep_release_fid_resources(ninep_fid_t *fid)
{
    if (fid->opened && fid->node.kind == NINEP_NODE_CONSOLE && s_console_open_count > 0) {
        portENTER_CRITICAL(&s_console_lock);
        if (s_console_open_count > 0) {
            --s_console_open_count;
        }
        portEXIT_CRITICAL(&s_console_lock);
    }

    if (fid->lcd_buffer != NULL) {
        heap_caps_free(fid->lcd_buffer);
        fid->lcd_buffer = NULL;
    }

    fid->opened = false;
    fid->open_mode = P9_OREAD;
    fid->lcd_fill = 0;
    fid->lcd_capacity = 0;
}

static void ninep_destroy_fid(ninep_client_t *client, uint32_t fid)
{
    ninep_fid_t **link = &client->fids;

    while (*link != NULL) {
        if ((*link)->fid == fid) {
            ninep_fid_t *victim = *link;
            *link = victim->next;
            ninep_release_fid_resources(victim);
            free(victim);
            return;
        }
        link = &(*link)->next;
    }
}

static void ninep_destroy_all_fids(ninep_client_t *client)
{
    while (client->fids != NULL) {
        ninep_destroy_fid(client, client->fids->fid);
    }
}

static ninep_fid_t *ninep_create_fid(ninep_client_t *client, uint32_t fid)
{
    ninep_fid_t *state = calloc(1, sizeof(*state));
    if (state == NULL) {
        return NULL;
    }

    state->fid = fid;
    state->node = (ninep_node_t) { .kind = NINEP_NODE_ROOT };
    state->next = client->fids;
    client->fids = state;
    return state;
}

static void ninep_reader_init(ninep_reader_t *reader, const uint8_t *buf, size_t len)
{
    reader->buf = buf;
    reader->len = len;
    reader->off = 0;
    reader->ok = true;
}

static uint8_t ninep_read_u8(ninep_reader_t *reader)
{
    if (!reader->ok || reader->off + 1 > reader->len) {
        reader->ok = false;
        return 0;
    }

    return reader->buf[reader->off++];
}

static uint16_t ninep_read_u16(ninep_reader_t *reader)
{
    uint16_t value;

    if (!reader->ok || reader->off + 2 > reader->len) {
        reader->ok = false;
        return 0;
    }

    value = (uint16_t)reader->buf[reader->off] | ((uint16_t)reader->buf[reader->off + 1] << 8);
    reader->off += 2;
    return value;
}

static uint32_t ninep_read_u32(ninep_reader_t *reader)
{
    uint32_t value;

    if (!reader->ok || reader->off + 4 > reader->len) {
        reader->ok = false;
        return 0;
    }

    value = (uint32_t)reader->buf[reader->off] |
            ((uint32_t)reader->buf[reader->off + 1] << 8) |
            ((uint32_t)reader->buf[reader->off + 2] << 16) |
            ((uint32_t)reader->buf[reader->off + 3] << 24);
    reader->off += 4;
    return value;
}

static uint64_t ninep_read_u64(ninep_reader_t *reader)
{
    uint64_t value = 0;

    if (!reader->ok || reader->off + 8 > reader->len) {
        reader->ok = false;
        return 0;
    }

    for (int i = 0; i < 8; ++i) {
        value |= ((uint64_t)reader->buf[reader->off + i]) << (8 * i);
    }
    reader->off += 8;
    return value;
}

static const uint8_t *ninep_read_bytes(ninep_reader_t *reader, size_t len)
{
    const uint8_t *ptr;

    if (!reader->ok || reader->off + len > reader->len) {
        reader->ok = false;
        return NULL;
    }

    ptr = &reader->buf[reader->off];
    reader->off += len;
    return ptr;
}

static const char *ninep_read_string(ninep_reader_t *reader, uint16_t *len_out)
{
    uint16_t len = ninep_read_u16(reader);
    const char *ptr = (const char *)ninep_read_bytes(reader, len);

    if (len_out != NULL) {
        *len_out = len;
    }
    return ptr;
}

static void ninep_writer_init(ninep_writer_t *writer, uint8_t *buf, size_t cap, uint8_t type, uint16_t tag)
{
    writer->buf = buf;
    writer->cap = cap;
    writer->off = 0;
    writer->ok = true;

    if (cap < 7) {
        writer->ok = false;
        return;
    }

    memset(buf, 0, cap);
    writer->off = 4;
    writer->buf[writer->off++] = type;
    writer->buf[writer->off++] = (uint8_t)(tag & 0xff);
    writer->buf[writer->off++] = (uint8_t)(tag >> 8);
}

static void ninep_write_u8(ninep_writer_t *writer, uint8_t value)
{
    if (!writer->ok || writer->off + 1 > writer->cap) {
        writer->ok = false;
        return;
    }

    writer->buf[writer->off++] = value;
}

static void ninep_write_u16(ninep_writer_t *writer, uint16_t value)
{
    if (!writer->ok || writer->off + 2 > writer->cap) {
        writer->ok = false;
        return;
    }

    writer->buf[writer->off++] = (uint8_t)(value & 0xff);
    writer->buf[writer->off++] = (uint8_t)(value >> 8);
}

static void ninep_write_u32(ninep_writer_t *writer, uint32_t value)
{
    if (!writer->ok || writer->off + 4 > writer->cap) {
        writer->ok = false;
        return;
    }

    writer->buf[writer->off++] = (uint8_t)(value & 0xff);
    writer->buf[writer->off++] = (uint8_t)((value >> 8) & 0xff);
    writer->buf[writer->off++] = (uint8_t)((value >> 16) & 0xff);
    writer->buf[writer->off++] = (uint8_t)((value >> 24) & 0xff);
}

static void ninep_write_u64(ninep_writer_t *writer, uint64_t value)
{
    if (!writer->ok || writer->off + 8 > writer->cap) {
        writer->ok = false;
        return;
    }

    for (int i = 0; i < 8; ++i) {
        writer->buf[writer->off++] = (uint8_t)((value >> (8 * i)) & 0xff);
    }
}

static void ninep_write_bytes(ninep_writer_t *writer, const void *data, size_t len)
{
    if (!writer->ok || writer->off + len > writer->cap) {
        writer->ok = false;
        return;
    }

    memcpy(&writer->buf[writer->off], data, len);
    writer->off += len;
}

static void ninep_write_string(ninep_writer_t *writer, const char *value)
{
    size_t len = strlen(value);

    if (len > UINT16_MAX) {
        writer->ok = false;
        return;
    }

    ninep_write_u16(writer, (uint16_t)len);
    ninep_write_bytes(writer, value, len);
}

static void ninep_write_qid(ninep_writer_t *writer, const ninep_qid_t *qid)
{
    ninep_write_u8(writer, qid->type);
    ninep_write_u32(writer, qid->version);
    ninep_write_u64(writer, qid->path);
}

static void ninep_patch_u16(ninep_writer_t *writer, size_t offset, uint16_t value)
{
    if (offset + 2 > writer->cap) {
        writer->ok = false;
        return;
    }

    writer->buf[offset] = (uint8_t)(value & 0xff);
    writer->buf[offset + 1] = (uint8_t)(value >> 8);
}

static void ninep_patch_u32(ninep_writer_t *writer, size_t offset, uint32_t value)
{
    if (offset + 4 > writer->cap) {
        writer->ok = false;
        return;
    }

    writer->buf[offset] = (uint8_t)(value & 0xff);
    writer->buf[offset + 1] = (uint8_t)((value >> 8) & 0xff);
    writer->buf[offset + 2] = (uint8_t)((value >> 16) & 0xff);
    writer->buf[offset + 3] = (uint8_t)((value >> 24) & 0xff);
}

static size_t ninep_finish_response(ninep_writer_t *writer)
{
    if (!writer->ok || writer->off > writer->cap) {
        return 0;
    }

    ninep_patch_u32(writer, 0, (uint32_t)writer->off);
    return writer->off;
}

static size_t ninep_stat_size(const ninep_node_t *node)
{
    char name[16];
    size_t name_len;

    ninep_node_name(node, name, sizeof(name));
    name_len = strlen(name);

    return 2 + 2 + 4 + 13 + 4 + 4 + 4 + 8 +
           2 + name_len +
           2 + strlen(NINEP_UID) +
           2 + strlen(NINEP_GID) +
           2 + strlen(NINEP_MUID);
}

static size_t ninep_encode_stat(const ninep_node_t *node, uint8_t *dst, size_t cap)
{
    ninep_writer_t writer;
    size_t entry_size;
    char name[16];
    ninep_qid_t qid;

    if (cap < ninep_stat_size(node)) {
        return 0;
    }

    ninep_node_name(node, name, sizeof(name));
    ninep_writer_init(&writer, dst, cap, 0, 0);
    writer.off = 0;
    writer.ok = true;
    qid = ninep_node_qid(node);

    ninep_write_u16(&writer, 0);
    ninep_write_u16(&writer, 0);
    ninep_write_u32(&writer, 0);
    ninep_write_qid(&writer, &qid);
    ninep_write_u32(&writer, ninep_node_mode(node));
    ninep_write_u32(&writer, 0);
    ninep_write_u32(&writer, 0);
    ninep_write_u64(&writer, ninep_node_length(node));
    ninep_write_string(&writer, name);
    ninep_write_string(&writer, NINEP_UID);
    ninep_write_string(&writer, NINEP_GID);
    ninep_write_string(&writer, NINEP_MUID);

    if (!writer.ok) {
        return 0;
    }

    entry_size = writer.off;
    ninep_patch_u16(&writer, 0, (uint16_t)(entry_size - 2));
    return entry_size;
}

static size_t ninep_encode_dir_read(const ninep_node_t *node, uint64_t offset, uint32_t count, uint8_t *dst, size_t cap)
{
    ninep_node_t child;
    size_t written = 0;
    uint64_t dir_offset = 0;
    uint32_t max_bytes = min_size(count, cap);
    size_t child_size;
    uint8_t stat_buf[96];

    if (!ninep_node_is_dir(node)) {
        return 0;
    }

    if (node->kind == NINEP_NODE_ROOT) {
        ninep_node_t root_children[] = {
            { .kind = NINEP_NODE_CONSOLE },
            { .kind = NINEP_NODE_GPIO_DIR },
#if ESP32_AUDIO
            { .kind = NINEP_NODE_PCM_DIR },
#endif
#if ESP32_LCD
            { .kind = NINEP_NODE_LCD_DIR },
#endif
        };

        for (size_t i = 0; i < sizeof(root_children) / sizeof(root_children[0]); ++i) {
            child = root_children[i];
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }

        return written;
    }

    if (node->kind == NINEP_NODE_PCM_DIR) {
        ninep_node_t pcm_children[] = {
            { .kind = NINEP_NODE_PCM_DAC },
            { .kind = NINEP_NODE_PCM_RATE },
            { .kind = NINEP_NODE_PCM_BUFFER },
        };

        for (size_t i = 0; i < sizeof(pcm_children) / sizeof(pcm_children[0]); ++i) {
            child = pcm_children[i];
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }

        return written;
    }

    if (node->kind == NINEP_NODE_LCD_DIR) {
        ninep_node_t lcd_children[] = {
            { .kind = NINEP_NODE_LCD_FRAMEBUFFER },
            { .kind = NINEP_NODE_LCD_RGB },
            { .kind = NINEP_NODE_LCD_JPEG },
            { .kind = NINEP_NODE_LCD_SCALE },
            { .kind = NINEP_NODE_LCD_X },
            { .kind = NINEP_NODE_LCD_Y },
        };

        for (size_t i = 0; i < sizeof(lcd_children) / sizeof(lcd_children[0]); ++i) {
            child = lcd_children[i];
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }

        return written;
    }

    if (node->kind == NINEP_NODE_GPIO_DIR) {
        ninep_node_t gpio_children[] = {
            { .kind = NINEP_NODE_GPIO_MODE_DIR },
            { .kind = NINEP_NODE_GPIO_FLAGS_DIR },
            { .kind = NINEP_NODE_GPIO_PWM_DIR },
            { .kind = NINEP_NODE_GPIO_ADC_DIR },
        };

        for (size_t i = 0; i < sizeof(gpio_children) / sizeof(gpio_children[0]); ++i) {
            child = gpio_children[i];
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }

        for (int pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
            if (!ninep_gpio_is_exported(pin)) {
                continue;
            }

            child = (ninep_node_t) { .kind = NINEP_NODE_GPIO_PIN, .pin = pin };
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }
    }

    if (node->kind == NINEP_NODE_GPIO_MODE_DIR || node->kind == NINEP_NODE_GPIO_FLAGS_DIR ||
        node->kind == NINEP_NODE_GPIO_PWM_DIR || node->kind == NINEP_NODE_GPIO_ADC_DIR) {
        ninep_node_kind_t child_kind = NINEP_NODE_GPIO_MODE_PIN;

        if (node->kind == NINEP_NODE_GPIO_FLAGS_DIR) {
            child_kind = NINEP_NODE_GPIO_FLAGS_PIN;
        } else if (node->kind == NINEP_NODE_GPIO_PWM_DIR) {
            child_kind = NINEP_NODE_GPIO_PWM_PIN;
        } else if (node->kind == NINEP_NODE_GPIO_ADC_DIR) {
            child_kind = NINEP_NODE_GPIO_ADC_PIN;
        }

        for (int pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
            if (!ninep_gpio_is_exported(pin)) {
                continue;
            }

            child = (ninep_node_t) { .kind = child_kind, .pin = pin };
            child_size = ninep_encode_stat(&child, stat_buf, sizeof(stat_buf));
            if (offset > dir_offset && offset != dir_offset) {
                if (offset < dir_offset + child_size) {
                    return 0;
                }
            }
            if (offset >= dir_offset + child_size) {
                dir_offset += child_size;
                continue;
            }
            if (written + child_size > max_bytes) {
                break;
            }
            memcpy(dst + written, stat_buf, child_size);
            written += child_size;
            dir_offset += child_size;
        }
    }

    return written;
}

static void ninep_console_append(const char *data, size_t len)
{
    portENTER_CRITICAL(&s_console_lock);
    for (size_t i = 0; i < len; ++i) {
        s_console_ring[s_console_write_seq % NINEP_CONSOLE_BYTES] = data[i];
        ++s_console_write_seq;
        if (s_console_write_seq - s_console_start_seq > NINEP_CONSOLE_BYTES) {
            s_console_start_seq = s_console_write_seq - NINEP_CONSOLE_BYTES;
        }
    }
    portEXIT_CRITICAL(&s_console_lock);
}

static int ninep_log_vprintf(const char *fmt, va_list ap)
{
    int ret;

    if (s_console_open_count > 0) {
        char tmp[256];
        va_list copy;
        int len;

        va_copy(copy, ap);
        len = vsnprintf(tmp, sizeof(tmp), fmt, copy);
        va_end(copy);

        if (len > 0) {
            ninep_console_append(tmp, min_size((size_t)len, sizeof(tmp) - 1));
        }
    }

    ret = s_ninep.prev_vprintf != NULL ? s_ninep.prev_vprintf(fmt, ap) : vprintf(fmt, ap);
    return ret;
}

static size_t ninep_read_console(const ninep_fid_t *fid, uint64_t offset, uint32_t count, uint8_t *dst)
{
    uint64_t requested_seq = fid->console_open_seq + offset;
    uint64_t start_seq;
    uint64_t end_seq;
    size_t available;
    size_t to_copy;

    portENTER_CRITICAL(&s_console_lock);
    start_seq = s_console_start_seq;
    end_seq = s_console_write_seq;

    if (requested_seq < start_seq) {
        requested_seq = start_seq;
    }
    if (requested_seq > end_seq) {
        requested_seq = end_seq;
    }

    available = (size_t)(end_seq - requested_seq);
    to_copy = min_size((size_t)count, available);
    for (size_t i = 0; i < to_copy; ++i) {
        dst[i] = (uint8_t)s_console_ring[(requested_seq + i) % NINEP_CONSOLE_BYTES];
    }
    portEXIT_CRITICAL(&s_console_lock);

    return to_copy;
}

static size_t ninep_read_text_value(const char *value, uint64_t offset, uint32_t count, uint8_t *dst)
{
    size_t len = strlen(value);

    if (offset >= len) {
        return 0;
    }

    len -= offset;
    len = min_size(len, count);
    memcpy(dst, value + offset, len);
    return len;
}

static esp_err_t ninep_apply_gpio_config(int pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = ninep_gpio_bit(pin),
        .pull_up_en = ((ninep_gpio_flags_for_pin(pin) & NINEP_GPIO_FLAG_PULL_UP) != 0) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = ((ninep_gpio_flags_for_pin(pin) & NINEP_GPIO_FLAG_PULL_DOWN) != 0) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    bool want_input = ninep_gpio_effective_input(pin);
    bool want_output = ninep_gpio_effective_output(pin);
    bool open_drain = (ninep_gpio_flags_for_pin(pin) & NINEP_GPIO_FLAG_OPEN_DRAIN) != 0;

    if (ninep_gpio_mode_for_pin(pin) == NINEP_GPIO_MODE_RSVD) {
        cfg.mode = GPIO_MODE_DISABLE;
        return gpio_config(&cfg);
    }

    if (want_output && !GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (open_drain && want_output) {
        cfg.mode = want_input ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_OUTPUT_OD;
    } else if (want_input && want_output) {
        cfg.mode = GPIO_MODE_INPUT_OUTPUT;
    } else if (want_output) {
        cfg.mode = GPIO_MODE_OUTPUT;
    } else if (want_input) {
        cfg.mode = GPIO_MODE_INPUT;
    } else {
        cfg.mode = GPIO_MODE_DISABLE;
    }

    return gpio_config(&cfg);
}

static size_t ninep_read_gpio_pin(int pin, uint64_t offset, uint32_t count, uint8_t *dst, esp_err_t *err_out)
{
    char value[4];

    if (ninep_gpio_mode_for_pin(pin) != NINEP_GPIO_MODE_DIG) {
        *err_out = ESP_ERR_INVALID_STATE;
        return 0;
    }

    *err_out = ninep_apply_gpio_config(pin);
    if (*err_out != ESP_OK) {
        return 0;
    }

    snprintf(value, sizeof(value), "%d\n", gpio_get_level((gpio_num_t)pin) ? 1 : 0);
    return ninep_read_text_value(value, offset, count, dst);
}

static bool ninep_parse_gpio_value(const uint8_t *data, size_t len, int *level_out)
{
    size_t start = 0;
    size_t end = len;

    while (start < len && isspace((unsigned char)data[start])) {
        ++start;
    }
    while (end > start && isspace((unsigned char)data[end - 1])) {
        --end;
    }

    if (end == start + 1 && data[start] == '0') {
        *level_out = 0;
        return true;
    }
    if (end == start + 1 && data[start] == '1') {
        *level_out = 1;
        return true;
    }
    if ((end - start == 2 && strncasecmp((const char *)&data[start], "on", 2) == 0) ||
        (end - start == 4 && strncasecmp((const char *)&data[start], "high", 4) == 0)) {
        *level_out = 1;
        return true;
    }
    if ((end - start == 3 && strncasecmp((const char *)&data[start], "off", 3) == 0) ||
        (end - start == 3 && strncasecmp((const char *)&data[start], "low", 3) == 0)) {
        *level_out = 0;
        return true;
    }

    return false;
}

static esp_err_t ninep_write_gpio_pin(int pin, const uint8_t *data, size_t len)
{
    int level = 0;

    ESP_RETURN_ON_FALSE(ninep_gpio_mode_for_pin(pin) == NINEP_GPIO_MODE_DIG, ESP_ERR_INVALID_STATE, TAG,
                        "gpio pin is not in dig mode");
    ESP_RETURN_ON_FALSE(ninep_parse_gpio_value(data, len, &level), ESP_ERR_INVALID_ARG, TAG, "invalid gpio value");
    ESP_RETURN_ON_FALSE(ninep_gpio_effective_output(pin), ESP_ERR_INVALID_STATE, TAG, "gpio pin is not configured for output");
    ESP_RETURN_ON_ERROR(ninep_apply_gpio_config(pin), TAG, "gpio setup failed");
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)pin, level), TAG, "gpio write failed");
    return ESP_OK;
}

static bool ninep_parse_gpio_mode(const uint8_t *data, size_t len, ninep_gpio_mode_t *mode_out)
{
    size_t start = 0;
    size_t end = len;

    while (start < len && isspace((unsigned char)data[start])) {
        ++start;
    }
    while (end > start && isspace((unsigned char)data[end - 1])) {
        --end;
    }

    if (end - start == 3 && strncasecmp((const char *)&data[start], "dig", 3) == 0) {
        *mode_out = NINEP_GPIO_MODE_DIG;
        return true;
    }
    if (end - start == 3 && strncasecmp((const char *)&data[start], "pwm", 3) == 0) {
        *mode_out = NINEP_GPIO_MODE_PWM;
        return true;
    }
    if (end - start == 3 && strncasecmp((const char *)&data[start], "adc", 3) == 0) {
        *mode_out = NINEP_GPIO_MODE_ADC;
        return true;
    }
    if ((end - start == 4 && strncasecmp((const char *)&data[start], "rsvd", 4) == 0) ||
        (end - start == 8 && strncasecmp((const char *)&data[start], "reserved", 8) == 0)) {
        *mode_out = NINEP_GPIO_MODE_RSVD;
        return true;
    }

    return false;
}

static size_t ninep_read_gpio_mode(int pin, uint64_t offset, uint32_t count, uint8_t *dst)
{
    char value[8];

    snprintf(value, sizeof(value), "%s\n", ninep_gpio_mode_name(ninep_gpio_mode_for_pin(pin)));
    return ninep_read_text_value(value, offset, count, dst);
}

static int ninep_adc_unit_index(adc_unit_t unit)
{
    switch (unit) {
    case ADC_UNIT_1:
        return 0;
    case ADC_UNIT_2:
        return 1;
    default:
        return -1;
    }
}

static esp_err_t ninep_validate_gpio_mode(int pin, ninep_gpio_mode_t mode)
{
    if (mode == NINEP_GPIO_MODE_PWM) {
        return GPIO_IS_VALID_OUTPUT_GPIO(pin) ? ESP_OK : ESP_ERR_INVALID_ARG;
    }

    if (mode == NINEP_GPIO_MODE_ADC) {
        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        return adc_oneshot_io_to_channel(pin, &unit, &channel);
    }

    return ESP_OK;
}

static void ninep_release_pwm_pin(int pin)
{
    int channel_index = s_ninep.pwm_channel_for_pin[pin];

    if (channel_index < 0) {
        s_ninep.pwm_duty[pin] = 0;
        return;
    }

    ledc_stop(NINEP_GPIO_PWM_SPEED_MODE, (ledc_channel_t)channel_index, 0);
    s_ninep.pwm_channel_for_pin[pin] = -1;
    s_ninep.pwm_pin_for_channel[channel_index] = -1;
    s_ninep.pwm_duty[pin] = 0;
}

static esp_err_t ninep_write_gpio_mode(int pin, const uint8_t *data, size_t len)
{
    ninep_gpio_mode_t prev_mode = ninep_gpio_mode_for_pin(pin);
    ninep_gpio_mode_t new_mode;

    ESP_RETURN_ON_FALSE(ninep_parse_gpio_mode(data, len, &new_mode), ESP_ERR_INVALID_ARG, TAG, "invalid gpio mode");
    ESP_RETURN_ON_ERROR(ninep_validate_gpio_mode(pin, new_mode), TAG, "unsupported gpio mode");
    s_ninep.gpio_modes[pin] = (uint8_t)new_mode;
    esp_err_t err = ninep_apply_gpio_config(pin);
    if (err != ESP_OK) {
        s_ninep.gpio_modes[pin] = (uint8_t)prev_mode;
        return err;
    }

    if (prev_mode == NINEP_GPIO_MODE_PWM && new_mode != NINEP_GPIO_MODE_PWM) {
        ninep_release_pwm_pin(pin);
        err = ninep_apply_gpio_config(pin);
        if (err != ESP_OK) {
            s_ninep.gpio_modes[pin] = (uint8_t)prev_mode;
            return err;
        }
    }

    return ESP_OK;
}

static bool ninep_parse_gpio_flags(const uint8_t *data, size_t len, uint8_t *flags_out)
{
    uint8_t flags = 0;
    size_t i = 0;

    while (i < len) {
        char token[24];
        size_t tok_len = 0;

        while (i < len && (isspace((unsigned char)data[i]) || data[i] == ',' || data[i] == '|')) {
            ++i;
        }
        if (i >= len) {
            break;
        }

        while (i < len && !isspace((unsigned char)data[i]) && data[i] != ',' && data[i] != '|') {
            if (tok_len + 1 >= sizeof(token)) {
                return false;
            }
            token[tok_len++] = (char)tolower((unsigned char)data[i]);
            ++i;
        }
        token[tok_len] = '\0';

        if (tok_len == 0) {
            continue;
        }
        if (strcmp(token, "none") == 0 || strcmp(token, "clear") == 0) {
            flags = 0;
            continue;
        }
        if (strcmp(token, "input") == 0 || strcmp(token, "in") == 0) {
            flags |= NINEP_GPIO_FLAG_INPUT;
            continue;
        }
        if (strcmp(token, "output") == 0 || strcmp(token, "out") == 0) {
            flags |= NINEP_GPIO_FLAG_OUTPUT;
            continue;
        }
        if (strcmp(token, "open_drain") == 0 || strcmp(token, "open-drain") == 0 ||
            strcmp(token, "opendrain") == 0 || strcmp(token, "od") == 0) {
            flags |= NINEP_GPIO_FLAG_OPEN_DRAIN;
            continue;
        }
        if (strcmp(token, "pull_up") == 0 || strcmp(token, "pullup") == 0) {
            flags |= NINEP_GPIO_FLAG_PULL_UP;
            continue;
        }
        if (strcmp(token, "pull_down") == 0 || strcmp(token, "pulldown") == 0) {
            flags |= NINEP_GPIO_FLAG_PULL_DOWN;
            continue;
        }

        return false;
    }

    *flags_out = flags;
    return true;
}

static size_t ninep_read_gpio_flags(int pin, uint64_t offset, uint32_t count, uint8_t *dst)
{
    char value[64];
    ninep_format_gpio_flags(pin, value, sizeof(value));
    return ninep_read_text_value(value, offset, count, dst);
}

static esp_err_t ninep_write_gpio_flags(int pin, const uint8_t *data, size_t len)
{
    uint8_t prev_flags = ninep_gpio_flags_for_pin(pin);
    uint8_t new_flags = 0;

    ESP_RETURN_ON_FALSE(ninep_parse_gpio_flags(data, len, &new_flags), ESP_ERR_INVALID_ARG, TAG, "invalid gpio flags");
    s_ninep.gpio_flags[pin] = new_flags;
    esp_err_t err = ninep_apply_gpio_config(pin);
    if (err != ESP_OK) {
        s_ninep.gpio_flags[pin] = prev_flags;
        return err;
    }

    return ESP_OK;
}

static size_t ninep_read_pcm_rate(uint64_t offset, uint32_t count, uint8_t *dst)
{
    char value[16];

    snprintf(value, sizeof(value), "%" PRIu32 "\n", audio_get_sample_rate_hz());
    return ninep_read_text_value(value, offset, count, dst);
}

static bool ninep_parse_u32_text(const uint8_t *data, size_t len, uint32_t *value_out)
{
    size_t start = 0;
    size_t end = len;
    uint64_t value = 0;

    while (start < len && isspace((unsigned char)data[start])) {
        ++start;
    }
    while (end > start && isspace((unsigned char)data[end - 1])) {
        --end;
    }
    if (start == end) {
        return false;
    }

    for (size_t i = start; i < end; ++i) {
        if (!isdigit((unsigned char)data[i])) {
            return false;
        }
        value = (value * 10ULL) + (uint64_t)(data[i] - '0');
        if (value > UINT32_MAX) {
            return false;
        }
    }

    *value_out = (uint32_t)value;
    return true;
}

static esp_err_t ninep_ensure_pwm_timer(void)
{
    if (s_ninep.pwm_timer_ready) {
        return ESP_OK;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = NINEP_GPIO_PWM_SPEED_MODE,
        .duty_resolution = NINEP_GPIO_PWM_RESOLUTION,
        .timer_num = NINEP_GPIO_PWM_TIMER,
        .freq_hz = NINEP_GPIO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "failed to configure pwm timer");
    s_ninep.pwm_timer_ready = true;
    return ESP_OK;
}

static esp_err_t ninep_ensure_pwm_channel(int pin, ledc_channel_t *channel_out)
{
    int channel_index = s_ninep.pwm_channel_for_pin[pin];

    ESP_RETURN_ON_ERROR(ninep_ensure_pwm_timer(), TAG, "failed to initialize pwm timer");

    if (channel_index < 0) {
        for (int i = 0; i < LEDC_CHANNEL_MAX; ++i) {
            if (s_ninep.pwm_pin_for_channel[i] < 0) {
                channel_index = i;
                break;
            }
        }
    }

    ESP_RETURN_ON_FALSE(channel_index >= 0, ESP_ERR_NOT_FOUND, TAG, "no pwm channels available");

    ledc_channel_config_t channel_cfg = {
        .gpio_num = pin,
        .speed_mode = NINEP_GPIO_PWM_SPEED_MODE,
        .channel = (ledc_channel_t)channel_index,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = NINEP_GPIO_PWM_TIMER,
        .duty = s_ninep.pwm_duty[pin],
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };

    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_cfg), TAG, "failed to configure pwm channel");
    s_ninep.pwm_channel_for_pin[pin] = (int8_t)channel_index;
    s_ninep.pwm_pin_for_channel[channel_index] = (int8_t)pin;
    *channel_out = (ledc_channel_t)channel_index;
    return ESP_OK;
}

static size_t ninep_read_gpio_pwm(int pin, uint64_t offset, uint32_t count, uint8_t *dst, esp_err_t *err_out)
{
    char value[16];

    if (ninep_gpio_mode_for_pin(pin) != NINEP_GPIO_MODE_PWM) {
        *err_out = ESP_ERR_INVALID_STATE;
        return 0;
    }

    snprintf(value, sizeof(value), "%" PRIu32 "\n", s_ninep.pwm_duty[pin]);
    return ninep_read_text_value(value, offset, count, dst);
}

static esp_err_t ninep_write_gpio_pwm(int pin, const uint8_t *data, size_t len)
{
    uint32_t duty = 0;
    ledc_channel_t channel = LEDC_CHANNEL_0;

    ESP_RETURN_ON_FALSE(ninep_gpio_mode_for_pin(pin) == NINEP_GPIO_MODE_PWM, ESP_ERR_INVALID_STATE, TAG,
                        "gpio pin is not in pwm mode");
    ESP_RETURN_ON_FALSE(ninep_parse_u32_text(data, len, &duty), ESP_ERR_INVALID_ARG, TAG, "invalid pwm value");
    ESP_RETURN_ON_FALSE(duty <= NINEP_GPIO_PWM_MAX_DUTY, ESP_ERR_INVALID_ARG, TAG, "pwm duty out of range");
    ESP_RETURN_ON_ERROR(ninep_ensure_pwm_channel(pin, &channel), TAG, "failed to allocate pwm channel");
    ESP_RETURN_ON_ERROR(ledc_set_duty_and_update(NINEP_GPIO_PWM_SPEED_MODE, channel, duty, 0), TAG,
                        "failed to update pwm duty");
    s_ninep.pwm_duty[pin] = duty;
    return ESP_OK;
}

static esp_err_t ninep_ensure_adc_channel(int pin, adc_oneshot_unit_handle_t *unit_out, adc_channel_t *channel_out)
{
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_unit_init_cfg_t unit_cfg = { 0 };
    int unit_index;
    uint32_t channel_mask;

    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(pin, &unit, &channel), TAG, "gpio is not adc-capable");

    unit_index = ninep_adc_unit_index(unit);
    ESP_RETURN_ON_FALSE(unit_index >= 0, ESP_ERR_NOT_SUPPORTED, TAG, "unsupported adc unit");

    if (s_ninep.adc_units[unit_index] == NULL) {
        unit_cfg.unit_id = unit;
        ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_ninep.adc_units[unit_index]), TAG, "failed to create adc unit");
    }

    channel_mask = 1UL << (uint32_t)channel;
    if ((s_ninep.adc_channel_mask[unit_index] & channel_mask) == 0) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_ninep.adc_units[unit_index], channel, &channel_cfg), TAG,
                            "failed to configure adc channel");
        s_ninep.adc_channel_mask[unit_index] |= channel_mask;
    }

    *unit_out = s_ninep.adc_units[unit_index];
    *channel_out = channel;
    return ESP_OK;
}

static size_t ninep_read_gpio_adc(int pin, uint64_t offset, uint32_t count, uint8_t *dst, esp_err_t *err_out)
{
    adc_oneshot_unit_handle_t unit = NULL;
    adc_channel_t channel = ADC_CHANNEL_0;
    char value[16];
    int raw = 0;

    if (ninep_gpio_mode_for_pin(pin) != NINEP_GPIO_MODE_ADC) {
        *err_out = ESP_ERR_INVALID_STATE;
        return 0;
    }

    *err_out = ninep_ensure_adc_channel(pin, &unit, &channel);
    if (*err_out != ESP_OK) {
        return 0;
    }

    *err_out = adc_oneshot_read(unit, channel, &raw);
    if (*err_out != ESP_OK) {
        return 0;
    }

    snprintf(value, sizeof(value), "%d\n", raw);
    return ninep_read_text_value(value, offset, count, dst);
}

static esp_err_t ninep_write_pcm_rate(const uint8_t *data, size_t len)
{
    uint32_t sample_rate_hz = 0;

    ESP_RETURN_ON_FALSE(ninep_parse_u32_text(data, len, &sample_rate_hz), ESP_ERR_INVALID_ARG, TAG, "invalid pcm rate");
    return audio_set_sample_rate_hz(sample_rate_hz);
}

static size_t ninep_read_pcm_buffer(uint64_t offset, uint32_t count, uint8_t *dst)
{
    char value[16];

    snprintf(value, sizeof(value), "%" PRIu32 "\n", audio_get_buffered_ms());
    return ninep_read_text_value(value, offset, count, dst);
}

static bool ninep_parse_lcd_scale(const uint8_t *data, size_t len, lcd_scale_mode_t *mode_out)
{
    size_t start = 0;
    size_t end = len;

    while (start < len && isspace((unsigned char)data[start])) {
        ++start;
    }
    while (end > start && isspace((unsigned char)data[end - 1])) {
        --end;
    }

    if (end - start == 4 && strncasecmp((const char *)&data[start], "none", 4) == 0) {
        *mode_out = LCD_SCALE_NONE;
        return true;
    }
    if (end - start == 6 && strncasecmp((const char *)&data[start], "aspect", 6) == 0) {
        *mode_out = LCD_SCALE_ASPECT;
        return true;
    }
    if (end - start == 4 && strncasecmp((const char *)&data[start], "full", 4) == 0) {
        *mode_out = LCD_SCALE_FULL;
        return true;
    }

    return false;
}

static size_t ninep_read_lcd_scale(uint64_t offset, uint32_t count, uint8_t *dst)
{
    char value[16];

    snprintf(value, sizeof(value), "%s\n", ninep_lcd_scale_name((lcd_scale_mode_t)s_ninep.lcd_scale_mode));
    return ninep_read_text_value(value, offset, count, dst);
}

static esp_err_t ninep_write_lcd_scale(const uint8_t *data, size_t len)
{
    lcd_scale_mode_t mode = LCD_SCALE_NONE;

    ESP_RETURN_ON_FALSE(ninep_parse_lcd_scale(data, len, &mode), ESP_ERR_INVALID_ARG, TAG, "invalid lcd scale");
    s_ninep.lcd_scale_mode = (uint8_t)mode;
    return ESP_OK;
}

static size_t ninep_read_lcd_coord(uint16_t value, uint64_t offset, uint32_t count, uint8_t *dst)
{
    char text[16];

    snprintf(text, sizeof(text), "%" PRIu16 "\n", value);
    return ninep_read_text_value(text, offset, count, dst);
}

static esp_err_t ninep_write_lcd_coord(uint16_t *value, uint16_t limit, const uint8_t *data, size_t len)
{
    uint32_t parsed = 0;

    ESP_RETURN_ON_FALSE(ninep_parse_u32_text(data, len, &parsed), ESP_ERR_INVALID_ARG, TAG, "invalid lcd coordinate");
    ESP_RETURN_ON_FALSE(parsed < limit, ESP_ERR_INVALID_ARG, TAG, "lcd coordinate out of range");
    *value = (uint16_t)parsed;
    return ESP_OK;
}

static esp_err_t ninep_reserve_lcd_buffer(ninep_fid_t *fid, size_t need)
{
    size_t new_capacity = fid->lcd_capacity;
    uint8_t *new_buffer;

    if (need <= fid->lcd_capacity) {
        return ESP_OK;
    }

    if (new_capacity == 0) {
        new_capacity = 4096;
    }
    while (new_capacity < need) {
        new_capacity *= 2;
    }

#if CONFIG_IDF_TARGET_ESP32S3
    new_buffer = heap_caps_realloc(fid->lcd_buffer, new_capacity, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (new_buffer == NULL) {
        new_buffer = heap_caps_realloc(fid->lcd_buffer, new_capacity, MALLOC_CAP_8BIT);
    }
#else
    new_buffer = heap_caps_realloc(fid->lcd_buffer, new_capacity, MALLOC_CAP_8BIT);
#endif

    ESP_RETURN_ON_FALSE(new_buffer != NULL, ESP_ERR_NO_MEM, TAG, "failed to allocate lcd buffer");
    fid->lcd_buffer = new_buffer;
    fid->lcd_capacity = new_capacity;
    return ESP_OK;
}

static esp_err_t ninep_open_lcd_framebuffer_fid(ninep_fid_t *fid)
{
    size_t frame_bytes = lcd_framebuffer_bytes();

    if (frame_bytes == 0) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ninep_reserve_lcd_buffer(fid, frame_bytes), TAG, "failed to allocate lcd frame buffer");
    fid->lcd_fill = 0;
    return ESP_OK;
}

static esp_err_t ninep_write_lcd_framebuffer(ninep_fid_t *fid, uint64_t offset, const uint8_t *data, size_t len, uint32_t *written_out)
{
    size_t frame_bytes = lcd_framebuffer_bytes();
    size_t consumed = 0;

    ESP_RETURN_ON_FALSE(frame_bytes > 0, ESP_ERR_NOT_SUPPORTED, TAG, "lcd framebuffer unavailable");
    ESP_RETURN_ON_FALSE(fid->lcd_buffer != NULL, ESP_ERR_INVALID_STATE, TAG, "lcd file not initialized");
    ESP_RETURN_ON_FALSE(offset == fid->lcd_fill, ESP_ERR_INVALID_ARG, TAG, "lcd framebuffer writes must be sequential");

    while (consumed < len) {
        size_t chunk = min_size(frame_bytes - fid->lcd_fill, len - consumed);
        memcpy(fid->lcd_buffer + fid->lcd_fill, data + consumed, chunk);
        fid->lcd_fill += chunk;
        consumed += chunk;

        if (fid->lcd_fill == frame_bytes) {
            ESP_RETURN_ON_ERROR(lcd_write_framebuffer(fid->lcd_buffer, frame_bytes), TAG, "lcd write failed");
            fid->lcd_fill = 0;
        }
    }

    *written_out = (uint32_t)consumed;
    return ESP_OK;
}

static esp_err_t ninep_write_lcd_rgb(uint64_t offset, const uint8_t *data, size_t len)
{
    return lcd_draw_rgb32(s_ninep.lcd_cursor_x, s_ninep.lcd_cursor_y, (size_t)offset, data, len);
}

static esp_err_t ninep_write_lcd_jpeg(ninep_fid_t *fid, uint64_t offset, const uint8_t *data, size_t len)
{
    size_t end = 0;

    ESP_RETURN_ON_FALSE(offset <= SIZE_MAX - len, ESP_ERR_INVALID_ARG, TAG, "jpeg write overflow");
    end = (size_t)offset + len;
    ESP_RETURN_ON_ERROR(ninep_reserve_lcd_buffer(fid, end), TAG, "failed to grow jpeg buffer");
    memcpy(fid->lcd_buffer + offset, data, len);
    if (fid->lcd_fill < end) {
        fid->lcd_fill = end;
    }
    return ESP_OK;
}

static esp_err_t ninep_flush_lcd_jpeg(ninep_fid_t *fid)
{
    if (fid->node.kind != NINEP_NODE_LCD_JPEG || fid->lcd_fill == 0) {
        return ESP_OK;
    }

    esp_err_t err = lcd_draw_jpeg(s_ninep.lcd_cursor_x, s_ninep.lcd_cursor_y,
                                  (lcd_scale_mode_t)s_ninep.lcd_scale_mode,
                                  fid->lcd_buffer, fid->lcd_fill);
    if (err == ESP_OK) {
        fid->lcd_fill = 0;
    }
    return err;
}

static esp_err_t ninep_recv_all(int sock, uint8_t *buf, size_t len)
{
    size_t received = 0;

    while (received < len) {
        int ret = recv(sock, buf + received, len - received, 0);
        if (ret == 0) {
            return ESP_ERR_INVALID_STATE;
        }
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            ESP_LOGW(TAG, "recv failed: errno=%d", errno);
            return ESP_FAIL;
        }
        received += (size_t)ret;
    }

    return ESP_OK;
}

static esp_err_t ninep_send_all(int sock, const uint8_t *buf, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        int ret = send(sock, buf + sent, len - sent, 0);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            ESP_LOGW(TAG, "send failed: errno=%d", errno);
            return ESP_FAIL;
        }
        sent += (size_t)ret;
    }

    return ESP_OK;
}

static size_t ninep_build_error_response(ninep_client_t *client, uint16_t tag, const char *message)
{
    ninep_writer_t writer;

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RERROR, tag);
    ninep_write_string(&writer, message);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_version(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    const char *version;
    uint16_t version_len = 0;
    uint32_t requested_msize = ninep_read_u32(reader);
    uint32_t negotiated_msize;
    ninep_writer_t writer;

    version = ninep_read_string(reader, &version_len);
    if (!reader->ok || version == NULL) {
        return ninep_build_error_response(client, tag, "bad version request");
    }

    negotiated_msize = requested_msize;
    if (negotiated_msize > NINEP_MAX_MSIZE) {
        negotiated_msize = NINEP_MAX_MSIZE;
    }
    if (negotiated_msize < NINEP_MIN_MSIZE) {
        negotiated_msize = NINEP_MIN_MSIZE;
    }

    client->msize = negotiated_msize;
    ninep_destroy_all_fids(client);

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RVERSION, tag);
    ninep_write_u32(&writer, negotiated_msize);
    if (version_len == strlen(NINEP_VERSION) && strncmp(version, NINEP_VERSION, version_len) == 0) {
        ninep_write_string(&writer, NINEP_VERSION);
    } else {
        ninep_write_string(&writer, "unknown");
    }
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_attach(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    ninep_writer_t writer;
    ninep_fid_t *state;
    ninep_qid_t qid;

    (void)ninep_read_u32(reader);
    (void)ninep_read_string(reader, NULL);
    (void)ninep_read_string(reader, NULL);

    if (!reader->ok) {
        return ninep_build_error_response(client, tag, "bad attach request");
    }
    if (ninep_find_fid(client, fid) != NULL) {
        return ninep_build_error_response(client, tag, "fid already in use");
    }

    state = ninep_create_fid(client, fid);
    if (state == NULL) {
        return ninep_build_error_response(client, tag, "no memory");
    }
    state->node = (ninep_node_t) { .kind = NINEP_NODE_ROOT };
    qid = ninep_node_qid(&state->node);

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RATTACH, tag);
    ninep_write_qid(&writer, &qid);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_walk(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    uint32_t newfid = ninep_read_u32(reader);
    uint16_t nwname = ninep_read_u16(reader);
    ninep_fid_t *from;
    ninep_fid_t *target;
    ninep_node_t node;
    ninep_qid_t qids[16];
    uint16_t walked = 0;
    ninep_writer_t writer;

    if (!reader->ok || nwname > 16) {
        return ninep_build_error_response(client, tag, "bad walk request");
    }

    from = ninep_find_fid(client, fid);
    if (from == NULL) {
        return ninep_build_error_response(client, tag, "unknown fid");
    }
    if (from->opened) {
        return ninep_build_error_response(client, tag, "cannot walk opened fid");
    }

    if (newfid != fid && ninep_find_fid(client, newfid) != NULL) {
        return ninep_build_error_response(client, tag, "newfid already in use");
    }

    node = from->node;
    for (uint16_t i = 0; i < nwname; ++i) {
        uint16_t name_len = 0;
        const char *name = ninep_read_string(reader, &name_len);
        ninep_node_t next;

        if (!reader->ok || name == NULL) {
            return ninep_build_error_response(client, tag, "bad walk name");
        }

        if (!ninep_walk_one(&node, name, name_len, &next)) {
            if (walked == 0) {
                return ninep_build_error_response(client, tag, "walk failed");
            }
            break;
        }

        node = next;
        qids[walked++] = ninep_node_qid(&node);
    }

    if (newfid == fid) {
        target = from;
    } else {
        target = ninep_create_fid(client, newfid);
        if (target == NULL) {
            return ninep_build_error_response(client, tag, "no memory");
        }
    }

    ninep_release_fid_resources(target);
    target->node = node;

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RWALK, tag);
    ninep_write_u16(&writer, walked);
    for (uint16_t i = 0; i < walked; ++i) {
        ninep_write_qid(&writer, &qids[i]);
    }
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_open(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    uint8_t mode = ninep_read_u8(reader);
    ninep_fid_t *state = ninep_find_fid(client, fid);
    ninep_writer_t writer;
    uint8_t open_mode = ninep_base_open_mode(mode);
    ninep_qid_t qid;

    if (!reader->ok || state == NULL) {
        return ninep_build_error_response(client, tag, "unknown fid");
    }
    if (state->opened) {
        return ninep_build_error_response(client, tag, "fid already opened");
    }
    if (ninep_node_is_dir(&state->node) && open_mode != P9_OREAD) {
        return ninep_build_error_response(client, tag, "directories are read-only");
    }
    if ((open_mode == P9_OREAD || open_mode == P9_OEXEC) && !ninep_node_can_read(&state->node)) {
        return ninep_build_error_response(client, tag, "file is write-only");
    }
    if ((open_mode == P9_OWRITE || open_mode == P9_ORDWR) && !ninep_node_can_write(&state->node)) {
        return ninep_build_error_response(client, tag, "file is read-only");
    }

    state->opened = true;
    state->open_mode = open_mode;

    if (state->node.kind == NINEP_NODE_CONSOLE) {
        portENTER_CRITICAL(&s_console_lock);
        state->console_open_seq = s_console_write_seq;
        ++s_console_open_count;
        portEXIT_CRITICAL(&s_console_lock);
    } else if (state->node.kind == NINEP_NODE_LCD_FRAMEBUFFER) {
        esp_err_t err = ninep_open_lcd_framebuffer_fid(state);
        if (err != ESP_OK) {
            state->opened = false;
            return ninep_build_error_response(client, tag, "lcd buffer allocation failed");
        }
    }

    qid = ninep_node_qid(&state->node);
    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_ROPEN, tag);
    ninep_write_qid(&writer, &qid);
    ninep_write_u32(&writer, client->msize > 24 ? client->msize - 24 : 0);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_read(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    uint64_t offset = ninep_read_u64(reader);
    uint32_t count = ninep_read_u32(reader);
    ninep_fid_t *state = ninep_find_fid(client, fid);
    ninep_writer_t writer;
    size_t data_offset;
    size_t data_len = 0;
    uint8_t *data_ptr;
    esp_err_t read_err = ESP_OK;

    if (!reader->ok || state == NULL || !state->opened) {
        return ninep_build_error_response(client, tag, "bad read fid");
    }
    if (state->open_mode == P9_OWRITE) {
        return ninep_build_error_response(client, tag, "file not opened for reading");
    }

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RREAD, tag);
    data_offset = writer.off;
    ninep_write_u32(&writer, 0);
    data_ptr = &writer.buf[writer.off];

    if (ninep_node_is_dir(&state->node)) {
        data_len = ninep_encode_dir_read(&state->node, offset, count, data_ptr, writer.cap - writer.off);
    } else if (state->node.kind == NINEP_NODE_CONSOLE) {
        data_len = ninep_read_console(state, offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_GPIO_PIN) {
        data_len = ninep_read_gpio_pin(state->node.pin, offset, count, data_ptr, &read_err);
    } else if (state->node.kind == NINEP_NODE_GPIO_MODE_PIN) {
        data_len = ninep_read_gpio_mode(state->node.pin, offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_GPIO_FLAGS_PIN) {
        data_len = ninep_read_gpio_flags(state->node.pin, offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_GPIO_PWM_PIN) {
        data_len = ninep_read_gpio_pwm(state->node.pin, offset, count, data_ptr, &read_err);
    } else if (state->node.kind == NINEP_NODE_GPIO_ADC_PIN) {
        data_len = ninep_read_gpio_adc(state->node.pin, offset, count, data_ptr, &read_err);
    } else if (state->node.kind == NINEP_NODE_PCM_RATE) {
        data_len = ninep_read_pcm_rate(offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_PCM_BUFFER) {
        data_len = ninep_read_pcm_buffer(offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_LCD_SCALE) {
        data_len = ninep_read_lcd_scale(offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_LCD_X) {
        data_len = ninep_read_lcd_coord(s_ninep.lcd_cursor_x, offset, count, data_ptr);
    } else if (state->node.kind == NINEP_NODE_LCD_Y) {
        data_len = ninep_read_lcd_coord(s_ninep.lcd_cursor_y, offset, count, data_ptr);
    } else {
        return ninep_build_error_response(client, tag, "file is not readable");
    }

    if (read_err != ESP_OK) {
        return ninep_build_error_response(client, tag, "gpio read failed");
    }

    writer.off += data_len;
    ninep_patch_u32(&writer, data_offset, (uint32_t)data_len);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_write(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    uint64_t offset = ninep_read_u64(reader);
    uint32_t count = ninep_read_u32(reader);
    const uint8_t *data = ninep_read_bytes(reader, count);
    ninep_fid_t *state = ninep_find_fid(client, fid);
    ninep_writer_t writer;
    uint32_t written = 0;

    if (!reader->ok || state == NULL || !state->opened || data == NULL) {
        return ninep_build_error_response(client, tag, "bad write fid");
    }
    if (state->open_mode == P9_OREAD || state->open_mode == P9_OEXEC) {
        return ninep_build_error_response(client, tag, "file not opened for writing");
    }

    if (state->node.kind == NINEP_NODE_GPIO_PIN) {
        esp_err_t err = ninep_write_gpio_pin(state->node.pin, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid gpio write");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_GPIO_MODE_PIN) {
        esp_err_t err = ninep_write_gpio_mode(state->node.pin, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid gpio mode");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_GPIO_FLAGS_PIN) {
        esp_err_t err = ninep_write_gpio_flags(state->node.pin, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid gpio flags");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_GPIO_PWM_PIN) {
        esp_err_t err = ninep_write_gpio_pwm(state->node.pin, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid pwm write");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_LCD_FRAMEBUFFER) {
        esp_err_t err = ninep_write_lcd_framebuffer(state, offset, data, count, &written);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "lcd framebuffer write failed");
        }
    } else if (state->node.kind == NINEP_NODE_LCD_RGB) {
        esp_err_t err = ninep_write_lcd_rgb(offset, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "lcd rgb write failed");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_LCD_JPEG) {
        esp_err_t err = ninep_write_lcd_jpeg(state, offset, data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "lcd jpeg write failed");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_LCD_SCALE) {
        esp_err_t err = ninep_write_lcd_scale(data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid lcd scale");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_LCD_X) {
        esp_err_t err = ninep_write_lcd_coord(&s_ninep.lcd_cursor_x, lcd_width(), data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid lcd x");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_LCD_Y) {
        esp_err_t err = ninep_write_lcd_coord(&s_ninep.lcd_cursor_y, lcd_height(), data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid lcd y");
        }
        written = count;
    } else if (state->node.kind == NINEP_NODE_PCM_DAC) {
        size_t audio_written = 0;
        esp_err_t err = audio_write(data, count, pdMS_TO_TICKS(1000), &audio_written);
        if (audio_written == 0 && err != ESP_OK) {
            return ninep_build_error_response(client, tag, "pcm write failed");
        }
        written = (uint32_t)audio_written;
    } else if (state->node.kind == NINEP_NODE_PCM_RATE) {
        esp_err_t err = ninep_write_pcm_rate(data, count);
        if (err != ESP_OK) {
            return ninep_build_error_response(client, tag, "invalid pcm rate");
        }
        written = count;
    } else {
        return ninep_build_error_response(client, tag, "file is not writable");
    }

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RWRITE, tag);
    ninep_write_u32(&writer, written);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_clunk(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    ninep_fid_t *state = ninep_find_fid(client, fid);
    ninep_writer_t writer;

    if (!reader->ok) {
        return ninep_build_error_response(client, tag, "bad clunk request");
    }

    if (state != NULL) {
        esp_err_t err = ninep_flush_lcd_jpeg(state);
        if (err != ESP_OK) {
            ninep_destroy_fid(client, fid);
            return ninep_build_error_response(client, tag, "lcd jpeg flush failed");
        }
    }

    ninep_destroy_fid(client, fid);
    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RCLUNK, tag);
    return ninep_finish_response(&writer);
}

static size_t ninep_handle_stat(ninep_client_t *client, uint16_t tag, ninep_reader_t *reader)
{
    uint32_t fid = ninep_read_u32(reader);
    ninep_fid_t *state = ninep_find_fid(client, fid);
    ninep_writer_t writer;
    uint8_t stat_buf[96];
    size_t stat_len;

    if (!reader->ok || state == NULL) {
        return ninep_build_error_response(client, tag, "unknown fid");
    }

    stat_len = ninep_encode_stat(&state->node, stat_buf, sizeof(stat_buf));
    if (stat_len == 0) {
        return ninep_build_error_response(client, tag, "stat encoding failed");
    }

    ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RSTAT, tag);
    ninep_write_u16(&writer, (uint16_t)stat_len);
    ninep_write_bytes(&writer, stat_buf, stat_len);
    return ninep_finish_response(&writer);
}

static size_t ninep_dispatch(ninep_client_t *client, uint8_t type, uint16_t tag, const uint8_t *body, size_t body_len)
{
    ninep_reader_t reader;

    ninep_reader_init(&reader, body, body_len);

    switch (type) {
    case P9_TVERSION:
        return ninep_handle_version(client, tag, &reader);
    case P9_TAUTH:
        return ninep_build_error_response(client, tag, "authentication not required");
    case P9_TATTACH:
        return ninep_handle_attach(client, tag, &reader);
    case P9_TFLUSH: {
        ninep_writer_t writer;
        (void)ninep_read_u16(&reader);
        ninep_writer_init(&writer, client->tx_buf, client->msize, P9_RFLUSH, tag);
        return ninep_finish_response(&writer);
    }
    case P9_TWALK:
        return ninep_handle_walk(client, tag, &reader);
    case P9_TOPEN:
        return ninep_handle_open(client, tag, &reader);
    case P9_TCREATE:
        return ninep_build_error_response(client, tag, "create not supported");
    case P9_TREAD:
        return ninep_handle_read(client, tag, &reader);
    case P9_TWRITE:
        return ninep_handle_write(client, tag, &reader);
    case P9_TCLUNK:
        return ninep_handle_clunk(client, tag, &reader);
    case P9_TREMOVE:
        return ninep_build_error_response(client, tag, "remove not supported");
    case P9_TSTAT:
        return ninep_handle_stat(client, tag, &reader);
    case P9_TWSTAT:
        return ninep_build_error_response(client, tag, "wstat not supported");
    default:
        return ninep_build_error_response(client, tag, "unsupported message");
    }
}

static void ninep_configure_client_socket(int sock)
{
    int keepalive = 1;
    int keepidle = 10;
    int keepinterval = 5;
    int keepcount = 3;

    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepinterval, sizeof(keepinterval));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcount, sizeof(keepcount));
}

static int ninep_create_listen_socket(uint16_t port)
{
    int listen_sock;
    int opt = 1;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return -1;
    }

    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed on port %u: errno=%d", port, errno);
        close(listen_sock);
        return -1;
    }

    if (listen(listen_sock, NINEP_LISTEN_BACKLOG) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(listen_sock);
        return -1;
    }

    ESP_LOGI(TAG, "9P server listening on tcp/%u with %d exported GPIOs", port, ninep_gpio_count());
    return listen_sock;
}

static void ninep_handle_client(int sock)
{
    ninep_client_t client = {
        .sock = sock,
        .msize = NINEP_DEFAULT_MSIZE,
    };

    client.rx_buf = heap_caps_malloc(NINEP_MAX_MSIZE, MALLOC_CAP_8BIT);
    client.tx_buf = heap_caps_malloc(NINEP_MAX_MSIZE, MALLOC_CAP_8BIT);
    if (client.rx_buf == NULL || client.tx_buf == NULL) {
        ESP_LOGE(TAG, "failed to allocate 9P buffers");
        goto cleanup;
    }

    while (1) {
        uint8_t header[7];
        uint32_t message_size;
        uint8_t type;
        uint16_t tag;
        size_t response_len;

        if (ninep_recv_all(sock, header, sizeof(header)) != ESP_OK) {
            break;
        }

        message_size = (uint32_t)header[0] |
                       ((uint32_t)header[1] << 8) |
                       ((uint32_t)header[2] << 16) |
                       ((uint32_t)header[3] << 24);
        type = header[4];
        tag = (uint16_t)header[5] | ((uint16_t)header[6] << 8);

        if (message_size < sizeof(header) || message_size > NINEP_MAX_MSIZE) {
            ESP_LOGW(TAG, "invalid 9P message size: %" PRIu32, message_size);
            break;
        }

        memcpy(client.rx_buf, header, sizeof(header));
        if (message_size > sizeof(header) &&
            ninep_recv_all(sock, client.rx_buf + sizeof(header), message_size - sizeof(header)) != ESP_OK) {
            break;
        }

        response_len = ninep_dispatch(&client, type, tag, client.rx_buf + sizeof(header), message_size - sizeof(header));
        if (response_len == 0 || ninep_send_all(sock, client.tx_buf, response_len) != ESP_OK) {
            break;
        }
    }

cleanup:
    ninep_destroy_all_fids(&client);
    if (client.rx_buf != NULL) {
        heap_caps_free(client.rx_buf);
    }
    if (client.tx_buf != NULL) {
        heap_caps_free(client.tx_buf);
    }
}

static void ninep_server_task(void *arg)
{
    (void)arg;

    while (1) {
        int listen_sock = ninep_create_listen_socket(s_ninep.tcp_port);
        if (listen_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        while (1) {
            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            char addr_str[64] = {0};
            int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

            if (sock < 0) {
                ESP_LOGW(TAG, "accept() failed: errno=%d", errno);
                break;
            }

            if (source_addr.ss_family == AF_INET) {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
            } else {
                snprintf(addr_str, sizeof(addr_str), "unknown");
            }

            ESP_LOGI(TAG, "9P client connected from %s", addr_str);
            ninep_configure_client_socket(sock);
            ninep_handle_client(sock);
            shutdown(sock, SHUT_RDWR);
            close(sock);
            ESP_LOGI(TAG, "9P client disconnected");
        }

        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

esp_err_t ninep_start(const ninep_config_t *config)
{
    uint16_t port = NINEP_DEFAULT_PORT;

    ESP_RETURN_ON_FALSE(!s_ninep.started, ESP_ERR_INVALID_STATE, TAG, "9P server already started");

    memset(&s_ninep, 0, sizeof(s_ninep));
    for (int pin = 0; pin < GPIO_PIN_COUNT; ++pin) {
        s_ninep.pwm_channel_for_pin[pin] = -1;
    }
    for (int channel = 0; channel < LEDC_CHANNEL_MAX; ++channel) {
        s_ninep.pwm_pin_for_channel[channel] = -1;
    }
    if (config != NULL) {
        if (config->tcp_port != 0) {
            port = config->tcp_port;
        }
        for (size_t i = 0; i < config->reserved_pin_count; ++i) {
            gpio_num_t pin = config->reserved_pins[i];
            if (pin != GPIO_NUM_NC && pin >= 0 && pin < 64) {
                s_ninep.reserved_gpio_mask |= ninep_gpio_bit(pin);
            }
        }
    }

    s_ninep.tcp_port = port;
    s_ninep.prev_vprintf = esp_log_set_vprintf(ninep_log_vprintf);

    BaseType_t task_ok = xTaskCreate(ninep_server_task, "ninep_tcp", NINEP_SERVER_TASK_STACK, NULL,
                                     NINEP_SERVER_TASK_PRIORITY, &s_ninep.server_task);
    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "failed to start 9P task");

    s_ninep.started = true;
    return ESP_OK;
}
