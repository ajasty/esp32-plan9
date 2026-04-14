#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

httpd_handle_t start_webserver(void);

#ifdef __cplusplus
}
#endif
