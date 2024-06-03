
#include "TempSensor.h"
#include <cmath>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <inttypes.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "esp_tls.h"
#include "sdkconfig.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE && CONFIG_EXAMPLE_USING_ESP_TLS_MBEDTLS
#include "esp_crt_bundle.h"
#endif
#include "time_sync.h"

#define WEB_SERVER "wttr.in"
#define WEB_PORT "443"
#define WEB_URL "https://wttr.in?format=%t&m"
#define WEB_PATH "/?format=%t&m"

#define SERVER_URL_MAX_SZ 256

static const char *TAG = "example";

#define TIME_PERIOD (86400000000ULL)

static const char HOWSMYSSL_REQUEST[] = "GET " WEB_PATH " HTTP/1.1\r\n"
                                        "Host: " WEB_SERVER "\r\n"
                                        "User-Agent: esp-idf/1.0 esp32\r\n"
                                        "\r\n";

static char https_response_body[512] = "";
static char http_response_body[512] = "";
static int roundedTemperature = 0;
static int roundedHumidity = 0;

extern const uint8_t
    server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t
    server_root_cert_pem_end[] asm("_binary_server_root_cert_pem_end");

extern const uint8_t
    local_server_cert_pem_start[] asm("_binary_local_server_cert_pem_start");
extern const uint8_t
    local_server_cert_pem_end[] asm("_binary_local_server_cert_pem_end");

#define PHONE_SERVER "10.0.0.179"
#define PHONE_PORT "1234"
#define PHONE_POST_URL "http://10.0.0.179:1234/store_data"
#define PHONE_POST_PATH "/store_data"

static const char *POST_REQUEST_FORMAT =
    "POST " PHONE_POST_PATH " HTTP/1.1\r\n"
    "Host: " PHONE_SERVER ":" PHONE_PORT "\r\n"
    "Content-Type: application/x-www-form-urlencoded\r\n"
    "Content-Length: %d\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n"
    "%s";

static void http_post_task(void) {
  const struct addrinfo hints = {
      .ai_family = AF_INET,
      .ai_socktype = SOCK_STREAM,
  };
  struct addrinfo *res;
  struct in_addr *addr;
  int s, r;
  char recv_buf[512];
  char post_data[256];

  // Construct the post_data with the global variables
  snprintf(post_data, sizeof(post_data),
           "wttrTemp=%s&localTemp=%d&localHumidity=%d&location=%s",
           https_response_body, roundedTemperature, roundedHumidity,
           http_response_body);

  int err = getaddrinfo(PHONE_SERVER, PHONE_PORT, &hints, &res);

  if (err != 0 || res == NULL) {
    ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
  ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

  s = socket(res->ai_family, res->ai_socktype, 0);
  if (s < 0) {
    ESP_LOGE(TAG, "... Failed to allocate socket.");
    freeaddrinfo(res);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... allocated socket");

  if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
    ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
    close(s);
    freeaddrinfo(res);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }

  ESP_LOGI(TAG, "... connected");
  freeaddrinfo(res);

  char request[1024];
  snprintf(request, sizeof(request), POST_REQUEST_FORMAT, strlen(post_data),
           post_data);

  if (write(s, request, strlen(request)) < 0) {
    ESP_LOGE(TAG, "... socket send failed");
    close(s);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... socket send success");

  struct timeval receiving_timeout;
  receiving_timeout.tv_sec = 5;
  receiving_timeout.tv_usec = 0;
  if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                 sizeof(receiving_timeout)) < 0) {
    ESP_LOGE(TAG, "... failed to set socket receiving timeout");
    close(s);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... set socket receiving timeout success");

  ESP_LOGI(TAG, "Reading HTTP response...");
  do {
    bzero(recv_buf, sizeof(recv_buf));
    r = read(s, recv_buf, sizeof(recv_buf) - 1);
    if (r < 0) {
      ESP_LOGE(TAG, "Error reading from socket errno=%d", errno);
      break;
    } else if (r == 0) {
      ESP_LOGI(TAG, "Connection closed");
      break;
    }

    recv_buf[r] = '\0'; // Ensure null-terminated string

    // Check if body has started
    char *body_start = strstr(recv_buf, "\r\n\r\n");
    if (body_start != NULL) {
      body_start += 4; // Move past the header delimiter
      ESP_LOGI(TAG, "Received HTTP response body:");
      ESP_LOGI(TAG, "%s", body_start);
      strcpy(http_response_body, body_start);
    } else {
      ESP_LOGI(TAG, "%s", recv_buf);
    }
  } while (r > 0);

  ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.",
           r, errno);
  close(s);
}
#define PHONE_URL "http://10.0.0.179:1234/location"
#define PHONE_PATH "/location"

static const char *REQUEST = "GET " PHONE_PATH " HTTP/1.1\r\n"
                             "Host: " PHONE_SERVER ":" PHONE_PORT "\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "\r\n";

static void http_get_task(void) {
  const struct addrinfo hints = {
      .ai_family = AF_INET,
      .ai_socktype = SOCK_STREAM,
  };
  struct addrinfo *res;
  struct in_addr *addr;
  int s, r;
  char recv_buf[512];

  int err = getaddrinfo(PHONE_SERVER, PHONE_PORT, &hints, &res);

  if (err != 0 || res == NULL) {
    ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
  ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

  s = socket(res->ai_family, res->ai_socktype, 0);
  if (s < 0) {
    ESP_LOGE(TAG, "... Failed to allocate socket.");
    freeaddrinfo(res);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... allocated socket");

  if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
    ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
    close(s);
    freeaddrinfo(res);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }

  ESP_LOGI(TAG, "... connected");
  freeaddrinfo(res);

  if (write(s, REQUEST, strlen(REQUEST)) < 0) {
    ESP_LOGE(TAG, "... socket send failed");
    close(s);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... socket send success");

  struct timeval receiving_timeout;
  receiving_timeout.tv_sec = 5;
  receiving_timeout.tv_usec = 0;
  if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                 sizeof(receiving_timeout)) < 0) {
    ESP_LOGE(TAG, "... failed to set socket receiving timeout");
    close(s);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }
  ESP_LOGI(TAG, "... set socket receiving timeout success");

  ESP_LOGI(TAG, "Reading HTTP response...");
  do {
    bzero(recv_buf, sizeof(recv_buf));
    r = read(s, recv_buf, sizeof(recv_buf) - 1);
    if (r < 0) {
      ESP_LOGE(TAG, "Error reading from socket errno=%d", errno);
      break;
    } else if (r == 0) {
      ESP_LOGI(TAG, "Connection closed");
      break;
    }

    recv_buf[r] = '\0'; // Ensure null-terminated string

    // Check if body has started
    char *body_start = strstr(recv_buf, "\r\n\r\n");
    if (body_start != NULL) {
      body_start += 4; // Move past the header delimiter
      ESP_LOGI(TAG, "Received HTTP response body:");
      ESP_LOGI(TAG, "%s", body_start);
      strcpy(http_response_body, body_start);
    } else {
      ESP_LOGI(TAG, "%s", recv_buf);
    }
  } while (r > 0);

  ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.",
           r, errno);
  close(s);
}
static void https_get_request(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL,
                              const char *REQUEST) {
  size_t written_bytes = 0;
  char buf[512];
  int ret, len;
  bool body_started = false;

  esp_tls_t *tls = esp_tls_init();
  if (!tls) {
    ESP_LOGE(TAG, "Failed to allocate esp_tls handle!");
    goto exit;
  }

  if (esp_tls_conn_http_new_sync(WEB_SERVER_URL, &cfg, tls) == 1) {
    ESP_LOGI(TAG, "Connection established...");
  } else {
    ESP_LOGE(TAG, "Connection failed...");
    int esp_tls_code = 0, esp_tls_flags = 0;
    esp_tls_error_handle_t tls_e = NULL;
    esp_tls_get_error_handle(tls, &tls_e);
    ret =
        esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
    if (ret == ESP_OK) {
      ESP_LOGE(TAG, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code,
               esp_tls_flags);
    }
    goto cleanup;
  }

  do {
    ret = esp_tls_conn_write(tls, REQUEST + written_bytes,
                             strlen(REQUEST) - written_bytes);
    if (ret >= 0) {
      ESP_LOGI(TAG, "%d bytes written", ret);
      written_bytes += ret;
    } else if (ret != ESP_TLS_ERR_SSL_WANT_READ &&
               ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
      ESP_LOGE(TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret,
               esp_err_to_name(ret));
      goto cleanup;
    }
  } while (written_bytes < strlen(REQUEST));

  ESP_LOGI(TAG, "Reading HTTP response...");
  do {
    len = sizeof(buf) - 1;
    memset(buf, 0x00, sizeof(buf));
    ret = esp_tls_conn_read(tls, (char *)buf, len);

    if (ret == ESP_TLS_ERR_SSL_WANT_WRITE || ret == ESP_TLS_ERR_SSL_WANT_READ) {
      continue;
    } else if (ret < 0) {
      ESP_LOGE(TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret,
               esp_err_to_name(ret));
      break;
    } else if (ret == 0) {
      ESP_LOGI(TAG, "connection closed");
      break;
    }

    len = ret;
    buf[len] = '\0';

    if (!body_started) {
      char *body_start = strstr(buf, "\r\n\r\n");
      if (body_start != NULL) {
        body_start += 4;
        body_started = true;
        ESP_LOGI(TAG, "Received HTTP response body:");
        ESP_LOGI(TAG, "%s", body_start);
        strcpy(https_response_body, body_start);
      }
    } else {
      ESP_LOGI(TAG, "%s", buf);
    }
  } while (1);

cleanup:
  esp_tls_conn_destroy(tls);
exit:
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void https_get_request_using_crt_bundle(void) {
  ESP_LOGI(TAG, "https_request using crt bundle");
  esp_tls_cfg_t cfg = {
      .crt_bundle_attach = esp_crt_bundle_attach,
  };
  https_get_request(cfg, WEB_URL, HOWSMYSSL_REQUEST);
}

static esp_err_t i2c_master_init() {
  i2c_port_t i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master = {.clk_speed = I2C_MASTER_FREQ_HZ},
  };
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "hal/i2c_types.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define SHTC3_SENSOR_ADDR 0x70
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

static void https_request_task(void *pvparameters) {
  ESP_LOGI(TAG, "Start https_request example");

  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");

  float temperature = 0;
  float humidity = 0;
  while (1) {
    http_get_task();
    https_get_request_using_crt_bundle();

    if (read_temperature(&temperature) == ESP_OK &&
        read_humidity(&humidity) == ESP_OK) {
      roundedTemperature = static_cast<int>(std::round(temperature));
      roundedHumidity = static_cast<int>(std::round(humidity));
      ESP_LOGI(TAG, "%d Celcius (local)", roundedTemperature);
      ESP_LOGI(TAG, "%d humidity", roundedHumidity);
    }
    http_post_task();

    ESP_LOGI(TAG, "waiting 5 seconds");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* This helper function configures Wi-Fi or Ethernet, as selected in
   * menuconfig. Read "Establishing Wi-Fi or Ethernet Connection" section in
   * examples/protocols/README.md for more information about this function.
   */
  ESP_ERROR_CHECK(example_connect());

  if (esp_reset_reason() == ESP_RST_POWERON) {
    ESP_LOGI(TAG, "Updating time from NVS");
    ESP_ERROR_CHECK(update_time_from_nvs());
  }

  const esp_timer_create_args_t nvs_update_timer_args = {
      .callback = (esp_timer_cb_t)&fetch_and_store_time_in_nvs,
  };

  esp_timer_handle_t nvs_update_timer;
  ESP_ERROR_CHECK(esp_timer_create(&nvs_update_timer_args, &nvs_update_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(nvs_update_timer, TIME_PERIOD));

  xTaskCreate(&https_request_task, "https_get_task", 8192, NULL, 5, NULL);
}
}
