#include "webserver.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "lcd.h"

typedef struct {
    uint16_t x, y, w, h;
} blit_req_t;

static const char *TAG = "webserver";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char resp[] = "hello world\n";

    httpd_resp_set_type(req, "text/plain");
    ESP_LOGI(TAG, "responding to http: hello world");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t color_get_handler(httpd_req_t *req)
{
    static const char resp[] = "setting color\n";

    httpd_resp_set_type(req, "text/plain");

    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen > 0) {
        char *qstr = malloc(qlen + 1);
        uint16_t color;

        if (qstr != NULL) {
            if (httpd_req_get_url_query_str(req, qstr, qlen + 1) == ESP_OK) {
                char val[64];

                if (httpd_query_key_value(qstr, "color", val, sizeof(val)) == ESP_OK) {
                    if (sscanf(val, "%4hx", &color) == 1) {
                        lcd_fill_rect(color);
                        ESP_LOGI(TAG, "color set to %x", color);
                    }
                }
            }
            free(qstr);
        }
    }

    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t blit_post_handler(httpd_req_t *req)
{
    blit_req_t bhdr;
    int payload_len;
    int malloc_caps = MALLOC_CAP_8BIT;

    if (httpd_req_recv(req, (char *)&bhdr, sizeof(bhdr)) != sizeof(bhdr)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    payload_len = bhdr.w * bhdr.h * 2;

    ESP_LOGW(TAG, "blit: %u %u %u %u %i", bhdr.x, bhdr.y, bhdr.w, bhdr.h, payload_len);

    if (bhdr.w == 0 || bhdr.w > 800 || bhdr.h == 0 || bhdr.h > 800) {
        ESP_LOGE(TAG, "Invalid dimensions: %i/%i", bhdr.w, bhdr.h);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

#if CONFIG_IDF_TARGET_ESP32S3
    malloc_caps |= MALLOC_CAP_SPIRAM;
#endif

    uint16_t *buf = heap_caps_malloc(payload_len, malloc_caps);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Alloc %u failed.", payload_len);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t left = payload_len;
    uint8_t *p = (uint8_t *)buf;
    while (left > 0) {
        int r = httpd_req_recv(req, (char *)p, left);
        if (r < 0) {
            ESP_LOGE(TAG, "recv err %d", r);
            heap_caps_free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        } else if (r == 0) {
            ESP_LOGW(TAG, "client closed early");
            heap_caps_free(buf);
            httpd_resp_send_408(req);
            return ESP_FAIL;
        }

        left -= r;
        p += r;
    }

    lcd_blit(bhdr.x, bhdr.y, bhdr.w, bhdr.h, buf);
    heap_caps_free(buf);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK\n");
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    config.server_port = 80;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL,
        };
        httpd_uri_t lcd = {
            .uri = "/lcd",
            .method = HTTP_GET,
            .handler = color_get_handler,
            .user_ctx = NULL,
        };
        const httpd_uri_t blit_uri = {
            .uri = "/blit",
            .method = HTTP_POST,
            .handler = blit_post_handler,
        };

        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &lcd);
        httpd_register_uri_handler(server, &blit_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }

    return server;
}
