// RepSense
// Adeildo Vieira (av259)

// Quality of Life includes for ESP-IDF
// OpenAI integration improvements
// Tests run and verified 12/13/2025 10:00 PM

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sdkconfig.h"

#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>

#include <stdint.h>

#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_http_client.h"
#include "esp_tls.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_http_client.h"
#include "esp_tls.h"

#include "esp32s3_box_lcd_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "RepSense";
static const char *TAG_OPENAI = "OpenAI";

// OpenAI connections:
#define OPENAI_BODY_MAX_LEN 2048

#ifndef CONFIG_OPENAI_API_KEY
#define CONFIG_OPENAI_API_KEY ""
#endif

static TaskHandle_t      g_openai_task_handle = NULL;
static SemaphoreHandle_t g_openai_mutex      = NULL;
static char              g_openai_body_buf[OPENAI_BODY_MAX_LEN];
static bool              g_openai_body_valid = false;

#define OPENAI_API_KEY CONFIG_OPENAI_API_KEY
#define OPENAI_MODEL "gpt-4o-mini" // using this one since 5 was not getting through

#define I2C_MASTER_SCL_IO          40
#define I2C_MASTER_SDA_IO          41
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TIMEOUT_MS      100

#define IMU_ADDR_DEFAULT           0x68

#define REG_SMPLRT_DIV             0x19
#define REG_CONFIG                 0x1A
#define REG_GYRO_CONFIG            0x1B
#define REG_ACCEL_CONFIG           0x1C
#define REG_ACCEL_CONFIG2          0x1D
#define REG_PWR_MGMT_1             0x6B
#define REG_WHO_AM_I               0x75
#define REG_ACCEL_XOUT_H           0x3B

// ±4 g -> 8192 LSB/g
#define ACCEL_LSB_PER_G_4G         8192.0f

#define BUTTON_GPIO                10

#define SAMPLE_RATE_HZ             100
#define SAMPLE_PERIOD_MS           (1000 / SAMPLE_RATE_HZ)

#define CALIB_SAMPLES              100   //using ~1 s of calibration at 100 Hz

// Acceleration (VERTICAL) for bench press mov.
#define VERT_SIGN_DEADZONE_G       0.02f   // tiny noise being ignored
#define VERT_MOTION_THRESH_G       0.02f   // starting rep if our |a_vert| >= this one
#define VERT_RETURN_THRESH_G       0.02f   // back to baseline of that specific person/set if |a_vert| <= this one

#define VERT_PEAK_MIN_G            0.075f   // it requires >= 0.075 g at some point in the rep


// WiFi configuration (same as Lab 4)

#define WIFI_SSID "DukeOpen"
#define WIFI_PASS ""

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT      = BIT1;

static bool g_wifi_ready = false;
static bool g_wifi_inited = false;

static const int WIFI_MAX_RETRY = 5;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WiFi", "retrying to connect to AP...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WiFi", "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_connect_dukeopen(void)
{
    if (g_wifi_ready) {
        return ESP_OK;
    }

    if (!g_wifi_inited) {
        s_wifi_event_group = xEventGroupCreate();

        esp_err_t err;

        err = esp_netif_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }

        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                                   ESP_EVENT_ANY_ID,
                                                   &wifi_event_handler,
                                                   NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                                   IP_EVENT_STA_GOT_IP,
                                                   &wifi_event_handler,
                                                   NULL));

        wifi_config_t wifi_config = {0};
        strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID,
                sizeof(wifi_config.sta.ssid));
        strlcpy((char *)wifi_config.sta.password, WIFI_PASS,
                sizeof(wifi_config.sta.password));

        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        g_wifi_inited = true;
    }

    // text stuff:
    ESP_LOGI("WiFi", "Connecting to SSID:%s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,      // do not clear bits on exit
        pdFALSE,      // wait for either bit
        pdMS_TO_TICKS(15000)  // 15s timeout
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WiFi", "connected to ap SSID:%s", WIFI_SSID);
        g_wifi_ready = true;
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW("WiFi", "failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL;
    } else {
        ESP_LOGW("WiFi", "Wi-Fi connect timeout");
        return ESP_FAIL;
    }
}

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG_OPENAI, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_ON_CONNECTED - TLS handshake successful");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_ON_HEADER: %s=%s",
                 evt->header_key ? evt->header_key : "(null)",
                 evt->header_value ? evt->header_value : "(null)");
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_ON_DATA, len=%d data=%.*s",
                 evt->data_len, 
                 evt->data_len > 100 ? 100 : evt->data_len,
                 (char *)evt->data);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT_DISCONNECTED");
        break;
    default:
        ESP_LOGI(TAG_OPENAI, "HTTP_EVENT: unknown event_id=%d", evt->event_id);
        break;
    }
    return ESP_OK;
}

static esp_err_t openai_send_chat_request(const char *json_body)
{
    if (!json_body) {
        ESP_LOGE(TAG_OPENAI, "No JSON body provided");
        return ESP_ERR_INVALID_ARG;
    }

    if (!OPENAI_API_KEY || strlen(OPENAI_API_KEY) == 0) {
        ESP_LOGE(TAG_OPENAI, "OpenAI API key is not set (configure CONFIG_OPENAI_API_KEY)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_wifi_ready) {
        ESP_LOGW(TAG_OPENAI, "Wi-Fi not ready; skipping OpenAI request");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG_OPENAI, "=== Sending request to OpenAI API ===");
    ESP_LOGI(TAG_OPENAI, "Waiting for AI response...");
    
    time_t now = time(NULL);
    struct tm *timeinfo = localtime(&now);
    ESP_LOGI(TAG_OPENAI, "System time: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    // HTTP client config for Chat Completions
    esp_http_client_config_t cfg = {
        .host                     = "api.openai.com",
        .path                     = "/v1/chat/completions",
        .method                   = HTTP_METHOD_POST,
        .event_handler            = _http_event_handler,
        .transport_type           = HTTP_TRANSPORT_OVER_SSL,

        // For DukeOpen network - disable strict SSL for now
        .skip_cert_common_name_check = true,
        .crt_bundle_attach        = esp_crt_bundle_attach,
        
        .timeout_ms               = 20000,
        .buffer_size              = 4096,
        .disable_auto_redirect    = true,
        .max_redirection_count    = 0,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG_OPENAI, "esp_http_client_init failed");
        return ESP_FAIL;
    }

    // Force HTTP/1.1 and set proper host header
    esp_http_client_set_header(client, "Host", "api.openai.com");
    
    // Headers
    char auth_header[300];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", OPENAI_API_KEY);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "User-Agent", "RepSense-ESP32");
    esp_http_client_set_header(client, "Connection", "close");
    
    // Debug: verify URL path
    char url_buf[256];
    esp_http_client_get_url(client, url_buf, sizeof(url_buf));
    ESP_LOGI(TAG_OPENAI, "Full URL being used: %s", url_buf);
    
    ESP_LOGI(TAG_OPENAI, "Authorization header set (key length: %d)", (int)strlen(OPENAI_API_KEY));

    int body_len = strlen(json_body);
    char content_length_str[32];
    snprintf(content_length_str, sizeof(content_length_str), "%d", body_len);
    esp_http_client_set_header(client, "Content-Length", content_length_str);
    esp_http_client_set_post_field(client, json_body, body_len);
    
    ESP_LOGI(TAG_OPENAI, "Sending OpenAI request...");

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OPENAI, "HTTP perform error: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    ESP_LOGI(TAG_OPENAI, "OpenAI HTTP status = %d, content_length = %d",
             status, content_length);

    char body[2048];
    memset(body, 0, sizeof(body));
    
    int total_read = 0;
    int r;
    while ((r = esp_http_client_read_response(client, body + total_read, sizeof(body) - total_read - 1)) > 0) {
        total_read += r;
        if (total_read >= (int)sizeof(body) - 1) {
            break;
        }
    }
    
    if (total_read > 0) {
        body[total_read] = '\0';
        ESP_LOGI(TAG_OPENAI, "Response received, total bytes: %d", total_read);
        
        if (total_read > 512) {
            ESP_LOGI(TAG_OPENAI, "Response body (first 512): %.512s", body);
            ESP_LOGI(TAG_OPENAI, "Response body (remaining): %.512s", body + 512);
            if (total_read > 1024) {
                ESP_LOGI(TAG_OPENAI, "Response body (end): %.512s", body + 1024);
            }
        } else {
            ESP_LOGI(TAG_OPENAI, "Response body: %s", body);
        }
    } else {
        ESP_LOGW(TAG_OPENAI, "No response body received (read returned %d), status=%d", r, status);
    }

    if (status / 100 != 2) {
        ESP_LOGW(TAG_OPENAI, "========== HTTP ERROR ==========");
        ESP_LOGW(TAG_OPENAI, "Status Code: %d", status);
        
        if (status >= 300 && status < 400) {
            char location_buf[256];
            if (esp_http_client_get_header(client, "Location", (char **)&location_buf) == ESP_OK) {
                ESP_LOGW(TAG_OPENAI, "Redirect to: %s", location_buf);
            }
        }
        
        ESP_LOGW(TAG_OPENAI, "Response: %s", total_read > 0 ? body : "(empty)");
        ESP_LOGW(TAG_OPENAI, "================================");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_OPENAI, "========== SUCCESS RESPONSE ==========");
    ESP_LOGI(TAG_OPENAI, "Response: %s", body);
    ESP_LOGI(TAG_OPENAI, "======================================");

    esp_http_client_cleanup(client);
    return ESP_OK;
}

static void openai_task(void *pv)
{
    (void)pv;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!g_openai_body_valid) {
            continue;
        }

        char local_body[OPENAI_BODY_MAX_LEN];
        local_body[0] = '\0';

        if (g_openai_mutex &&
            xSemaphoreTake(g_openai_mutex, portMAX_DELAY) == pdTRUE) {
            strncpy(local_body, g_openai_body_buf, sizeof(local_body) - 1);
            local_body[sizeof(local_body) - 1] = '\0';
            g_openai_body_valid = false;
            xSemaphoreGive(g_openai_mutex);
        } else {
            ESP_LOGW("OpenAI", "Failed to take mutex in openai_task");
            continue;
        }

        esp_err_t err = openai_send_chat_request(local_body);
        if (err != ESP_OK) {
            ESP_LOGW("OpenAI", "openai_send_chat_request failed (err=%s)",
                     esp_err_to_name(err));
        }
    }
}

static volatile bool     session_running   = false;
static volatile int64_t  session_start_us  = 0;

static volatile bool  baseline_ready       = false;
static float          baseline_acc_g       = 0.0f;
static float          g_unit_x             = 0.0f;
static float          g_unit_y             = 0.0f;
static float          g_unit_z             = 0.0f;

static float baseline_angle_deg = 0.0f;
static float last_angle_deg     = 0.0f;

static int   rep_count     = 0;

static bool  rep_in_motion    = false;
static int   direction_changes = 0;
static int   last_sign         = 0;

static float rep_peak_abs_vert = 0.0f;

static uint8_t g_imu_addr = IMU_ADDR_DEFAULT;

static lv_disp_t *disp         = NULL;
static lv_obj_t  *label_status = NULL;
static lv_obj_t  *label_time   = NULL;
static lv_obj_t  *label_reps   = NULL;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialized (SDA=%d, SCL=%d)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Starting I2C scan on port %d...", I2C_MASTER_NUM);
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                             pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device found at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan done");
}

static esp_err_t imu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      g_imu_addr,
                                      buf, sizeof(buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t imu_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        g_imu_addr,
                                        &reg, 1,
                                        data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t imu_init(void)
{
    i2c_scan();

    esp_err_t ret;
    uint8_t who = 0;

    g_imu_addr = 0x68;
    ret = imu_read_bytes(REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed at 0x68: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (addr 0x%02X)", who, g_imu_addr);
    if (who != 0x11 && who != 0x70) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I (expected 0x11 or 0x70)");
    } else {
        ESP_LOGI(TAG, "Accepted IMU WHO_AM_I");
    }

    ret = imu_write_reg(REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = imu_write_reg(REG_GYRO_CONFIG, 0x08);
    if (ret != ESP_OK) goto fail;

    ret = imu_write_reg(REG_ACCEL_CONFIG, 0x08);
    if (ret != ESP_OK) goto fail;

    ret = imu_write_reg(REG_ACCEL_CONFIG2, 0x0B);
    if (ret != ESP_OK) goto fail;

    ret = imu_write_reg(REG_SMPLRT_DIV, 9);
    if (ret != ESP_OK) goto fail;

    ret = imu_write_reg(REG_CONFIG, 0x03);
    if (ret != ESP_OK) goto fail;

    ESP_LOGI(TAG, "IMU initialized at 0x%02X", g_imu_addr);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "IMU init write failed: %s", esp_err_to_name(ret));
    return ret;
}

static esp_err_t imu_read_accel(float *ax_g, float *ay_g, float *az_g)
{
    uint8_t raw[6];
    esp_err_t ret = imu_read_bytes(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (ret != ESP_OK) return ret;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    *ax_g = (float)ax / ACCEL_LSB_PER_G_4G;
    *ay_g = (float)ay / ACCEL_LSB_PER_G_4G;
    *az_g = (float)az / ACCEL_LSB_PER_G_4G;

    return ESP_OK;
}

// ----------------------------------------------------
// Display / LVGL setup (like Lab 4)
// ----------------------------------------------------

static lv_disp_t *gui_setup(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    spi_bus_config_t bus_config = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .flags.reset_active_high = 1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
        .flags = {.swap_bytes = true},
        .rotation = {.swap_xy = false, .mirror_x = true, .mirror_y = true}
    };
    return lvgl_port_add_disp(&disp_cfg);
}

static void update_labels_idle(const char *reason)
{
    if (!label_status || !label_time || !label_reps) return;

    char buf[160];
    lvgl_port_lock(0);

    snprintf(buf, sizeof(buf),
             "RepSense: %s\nPress button to START", reason ? reason : "IDLE");
    lv_label_set_text(label_status, buf);

    lv_label_set_text(label_time, "Time: 0.0 s");
    lv_label_set_text(label_reps, "Reps: 0");

    lvgl_port_unlock();
}

static void update_labels_running(void)
{
    if (!label_status || !label_time || !label_reps) return;

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us  = now_us - session_start_us;
    if (dt_us < 0) dt_us = 0;
    float seconds = (float)dt_us / 1e6f;

    char buf[160];
    lvgl_port_lock(0);

    snprintf(buf, sizeof(buf),
             "RepSense: RUNNING\n(press button to STOP)");
    lv_label_set_text(label_status, buf);

    snprintf(buf, sizeof(buf), "Time: %.1f s", seconds);
    lv_label_set_text(label_time, buf);

    snprintf(buf, sizeof(buf), "Reps: %d", rep_count);
    lv_label_set_text(label_reps, buf);

    lvgl_port_unlock();
}

static void update_labels_done(float total_time_s, float imbalance_deg)
{
    if (!label_status || !label_time || !label_reps) return;

    char buf[160];
    lvgl_port_lock(0);

    snprintf(buf, sizeof(buf),
             "RepSense: DONE\nImbalance: %.1f deg\nPress button to START again",
             imbalance_deg);
    lv_label_set_text(label_status, buf);

    snprintf(buf, sizeof(buf), "Time: %.1f s", total_time_s);
    lv_label_set_text(label_time, buf);

    snprintf(buf, sizeof(buf), "Reps: %d", rep_count);
    lv_label_set_text(label_reps, buf);

    lvgl_port_unlock();
}

static void create_main_screen(void)
{
    lv_obj_t *scr = lv_scr_act();

    label_status = lv_label_create(scr);
    lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_long_mode(label_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_status, 220);

    label_time = lv_label_create(scr);
    lv_obj_align(label_time, LV_ALIGN_TOP_MID, 0, 70);

    label_reps = lv_label_create(scr);
    lv_obj_align(label_reps, LV_ALIGN_TOP_MID, 0, 110);

    update_labels_idle("READY");
}

typedef struct {
    uint32_t session_id;
    float    total_time_s;
    int      reps;
    float    imbalance_deg;
} repsense_session_t;

static uint32_t g_session_counter = 0;

static void build_session_toon(const repsense_session_t *s,
                               char *out,
                               size_t out_size)
{
    if (!s || !out || out_size == 0) {
        return;
    }

    snprintf(out, out_size,
             "RepSenseToon v1\n"
             "session_id: %lu\n"
             "time_s: %.2f\n"
             "reps: %d\n"
             "imbalance_deg: %.2f\n",
             (unsigned long)s->session_id,
             s->total_time_s,
             s->reps,
             s->imbalance_deg);
}

static void json_escape_string(const char *in, char *out, size_t out_size)
{
    if (!in || !out || out_size == 0) {
        return;
    }

    size_t o = 0;
    for (size_t i = 0; in[i] != '\0' && o + 2 < out_size; ++i) {
        char c = in[i];
        switch (c) {
        case '\"':
        case '\\':
            if (o + 2 >= out_size) {
                i = strlen(in);  // force exit
                break;
            }
            out[o++] = '\\';
            out[o++] = c;
            break;
        case '\n':
            if (o + 2 >= out_size) {
                i = strlen(in);
                break;
            }
            out[o++] = '\\';
            out[o++] = 'n';
            break;
        case '\r':
            if (o + 2 >= out_size) {
                i = strlen(in);
                break;
            }
            out[o++] = '\\';
            out[o++] = 'r';
            break;
        case '\t':
            if (o + 2 >= out_size) {
                i = strlen(in);
                break;
            }
            out[o++] = '\\';
            out[o++] = 't';
            break;
        default:
            out[o++] = c;
            break;
        }
    }
    out[o] = '\0';
}

static void build_openai_chat_body(const char *toon,
                                   char *out,
                                   size_t out_size)
{
    if (!toon || !out || out_size == 0) {
        return;
    }

    const char *system_prompt =
        "You are RepSense, an AI gym coach. "
        "The user will send you workout sets in a custom text format called 'RepSenseToon v1'. "
        "Each Toon includes: session_id, time_s, reps, and imbalance_deg. "
        "DO NOT HALLUCINATE or invent data. DO NOT ask questions. "
        "Only give short, concrete feedback based on these fields. "
        "1) Briefly assess the set (fatigue, pace, stability). "
        "2) Comment on imbalance_deg in simple terms (left/right balance). "
        "3) Suggest ONE short, practical tip for the next set. "
        "Keep your response to 3 bullet points, max.";

    char escaped_system[512];
    json_escape_string(system_prompt, escaped_system, sizeof(escaped_system));

    char escaped_toon[512];
    json_escape_string(toon, escaped_toon, sizeof(escaped_toon));

    snprintf(out, out_size,
             "{"
               "\"model\":\"%s\","
               "\"messages\":["
                 "{\"role\":\"system\",\"content\":\"%s\"},"
                 "{\"role\":\"user\",\"content\":\"%s\"}"
               "],"
               "\"max_tokens\":150"
             "}",
             OPENAI_MODEL,
             escaped_system,
             escaped_toon);
}

static void reset_rep_state(void)
{
    rep_count        = 0;
    rep_in_motion    = false;
    direction_changes = 0;
    last_sign         = 0;
}

static void start_session(void)
{
    if (session_running) return;

    ESP_LOGI(TAG, "Session START");

    baseline_ready    = false;
    baseline_acc_g    = 0.0f;
    g_unit_x = g_unit_y = g_unit_z = 0.0f;
    baseline_angle_deg = 0.0f;
    last_angle_deg     = 0.0f;

    reset_rep_state();

    session_start_us = esp_timer_get_time();
    session_running  = true;

    update_labels_idle("CALIBRATING...");
}

static void stop_session(void)
{
    if (!session_running) {
        return;
    }

    session_running = false;

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us  = now_us - session_start_us;
    if (dt_us < 0) {
        dt_us = 0;
    }
    float seconds = (float)dt_us / 1e6f;

    float imbalance_deg = fabsf(last_angle_deg - baseline_angle_deg);

    repsense_session_t sess = {
        .session_id     = ++g_session_counter,
        .total_time_s   = seconds,
        .reps           = rep_count,
        .imbalance_deg  = imbalance_deg,
    };

    char toon_buf[256];
    build_session_toon(&sess, toon_buf, sizeof(toon_buf));

    char openai_body[1024];
    build_openai_chat_body(toon_buf, openai_body, sizeof(openai_body));

    ESP_LOGI(TAG,
             "Session STOP: time = %.2f s, reps = %d, imbalance = %.2f deg",
             seconds, rep_count, imbalance_deg);
    ESP_LOGI(TAG, "Session TOON payload:\n%s", toon_buf);
    ESP_LOGI(TAG, "OpenAI Chat body:\n%s", openai_body);

    if (g_openai_task_handle && g_openai_mutex) {
        if (xSemaphoreTake(g_openai_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            strncpy(g_openai_body_buf, openai_body,
                    sizeof(g_openai_body_buf) - 1);
            g_openai_body_buf[sizeof(g_openai_body_buf) - 1] = '\0';
            g_openai_body_valid = true;
            xSemaphoreGive(g_openai_mutex);

            xTaskNotifyGive(g_openai_task_handle);
        } else {
            ESP_LOGW("OpenAI", "Could not take mutex to queue OpenAI request");
        }
    } else {
        ESP_LOGW("OpenAI", "OpenAI task not ready; skipping request");
    }

    update_labels_done(seconds, imbalance_deg);
}

static void rep_update_from_vertical(float a_vert)
{
    if (!baseline_ready || !session_running) return;

    float abs_vert = fabsf(a_vert);

    // Determine sign with deadzone
    int sign;
    if (a_vert > VERT_SIGN_DEADZONE_G) {
        sign = 1;
    } else if (a_vert < -VERT_SIGN_DEADZONE_G) {
        sign = -1;
    } else {
        sign = 0;
    }

    if (!rep_in_motion) {
        // Start of motion: |a_vert| big enough
        if (abs_vert >= VERT_MOTION_THRESH_G) {
            rep_in_motion      = true;
            direction_changes  = 0;
            last_sign          = sign;
            rep_peak_abs_vert  = abs_vert;   // start tracking peak
            ESP_LOGI(TAG, "Rep motion started (a_vert=%.3f g)", a_vert);
        }
    } else {
        // While in motion, update peak vertical accel
        if (abs_vert > rep_peak_abs_vert) {
            rep_peak_abs_vert = abs_vert;
        }

        if (sign != 0 && last_sign != 0 && sign != last_sign) {
            direction_changes++;
            ESP_LOGI(TAG, "Direction change %d (sign %d -> %d, a_vert=%.3f)",
                     direction_changes, last_sign, sign, a_vert);
            last_sign = sign;
        }

        if (direction_changes >= 2 && abs_vert <= VERT_RETURN_THRESH_G) {
            if (rep_peak_abs_vert >= VERT_PEAK_MIN_G) {
                rep_count++;
                ESP_LOGI(TAG, "Rep %d (peak |a_vert|=%.3f g)", rep_count, rep_peak_abs_vert);
            } else {
                ESP_LOGI(TAG,
                         "Motion ended but peak |a_vert|=%.3f g < %.3f g (not counting)",
                         rep_peak_abs_vert, VERT_PEAK_MIN_G);
            }

            rep_in_motion      = false;
            direction_changes  = 0;
            last_sign          = 0;
            rep_peak_abs_vert  = 0.0f;
        }
    }
}

static float compute_angle_deg(float ax_g, float ay_g, float az_g)
{
    float denom = sqrtf(ay_g * ay_g + az_g * az_g);
    if (denom < 1e-6f) denom = 1e-6f;
    float pitch_rad = atan2f(-ax_g, denom);
    return pitch_rad * 180.0f / (float)M_PI;
}

static void imu_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "IMU task started");

    int   calib_count   = 0;
    float calib_sum_ax  = 0.0f;
    float calib_sum_ay  = 0.0f;
    float calib_sum_az  = 0.0f;

    bool  prev_running  = false;
    int   ui_counter    = 0;
    int   dbg_counter   = 0;

    while (1) {
        if (!session_running) {
            prev_running = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!prev_running) {
            calib_count   = 0;
            calib_sum_ax  = 0.0f;
            calib_sum_ay  = 0.0f;
            calib_sum_az  = 0.0f;

            baseline_ready   = false;
            reset_rep_state();

            prev_running  = true;
            ESP_LOGI(TAG, "Calibration phase started");
        }

        float ax_g, ay_g, az_g;
        esp_err_t ret = imu_read_accel(&ax_g, &ay_g, &az_g);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read accel: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        if (!baseline_ready) {
            calib_sum_ax += ax_g;
            calib_sum_ay += ay_g;
            calib_sum_az += az_g;
            calib_count++;

            if (calib_count >= CALIB_SAMPLES) {
                float avg_ax = calib_sum_ax / (float)calib_count;
                float avg_ay = calib_sum_ay / (float)calib_count;
                float avg_az = calib_sum_az / (float)calib_count;

                baseline_acc_g = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);
                if (baseline_acc_g < 1e-6f) baseline_acc_g = 1e-6f;

                g_unit_x = avg_ax / baseline_acc_g;
                g_unit_y = avg_ay / baseline_acc_g;
                g_unit_z = avg_az / baseline_acc_g;

                baseline_angle_deg = compute_angle_deg(avg_ax, avg_ay, avg_az);
                last_angle_deg     = baseline_angle_deg;

                baseline_ready = true;
                ESP_LOGI(TAG,
                         "Baseline |acc| = %.3f g, g_unit=(%.3f, %.3f, %.3f), angle=%.2f deg",
                         baseline_acc_g, g_unit_x, g_unit_y, g_unit_z, baseline_angle_deg);
            }

        } else {
            float proj   = ax_g * g_unit_x + ay_g * g_unit_y + az_g * g_unit_z;
            float a_vert = proj - baseline_acc_g;

            last_angle_deg = compute_angle_deg(ax_g, ay_g, az_g);

            rep_update_from_vertical(a_vert);

            dbg_counter++;
            if (dbg_counter >= 10) {
                dbg_counter = 0;
                ESP_LOGI(TAG,
                        "a_vert=%.3f g, angle=%.2f deg, reps=%d, running=%d",
                        a_vert, last_angle_deg, rep_count, session_running ? 1 : 0);
            }
        }

        ui_counter++;
        if (ui_counter >= 10) {
            ui_counter = 0;
            update_labels_running();
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}




// button stuff:
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void button_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Button task started on GPIO %d", BUTTON_GPIO);

    int last_level = gpio_get_level(BUTTON_GPIO);

    while (1) {
        int level = gpio_get_level(BUTTON_GPIO);

        if (last_level == 1 && level == 0) {
            vTaskDelay(pdMS_TO_TICKS(30));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                if (!session_running) {
                    start_session();
                } else {
                    stop_session();
                }
            }
        }

        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    disp = gui_setup();

    lvgl_port_lock(0);
    create_main_screen();
    lvgl_port_unlock();

    update_labels_idle("INIT I2C/IMU...");

    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        update_labels_idle("I2C ERROR");
        return;
    }

    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(ret));
        update_labels_idle("IMU ERROR");
        return;
    }

    button_init();

    xTaskCreate(imu_task,    "imu_task",    4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 8192, NULL, 6, NULL);

    ESP_LOGI(TAG, "RepSense ready – press the button to START/STOP");

    ret = wifi_connect_dukeopen();
    if (ret != ESP_OK) {
        ESP_LOGW("WiFi",
                 "Could not connect to SSID:%s. "
                 "Continuing without Wi-Fi; OpenAI calls will be skipped.",
                 WIFI_SSID);
    } else {
        ESP_LOGI("WiFi", "Wi-Fi connected to %s, OpenAI ready", WIFI_SSID);
        
        ESP_LOGI(TAG, "Initializing SNTP for time sync...");
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_setservername(1, "time.google.com");
        esp_sntp_setservername(2, "time.nist.gov");
        esp_sntp_init();
        
        time_t now = 0;
        struct tm timeinfo = {0};
        int retry = 0;
        const int retry_count = 4; // 14 seconds
        
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
            ESP_LOGI(TAG, "Waiting for SNTP sync... (%d/%d)", retry, retry_count);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        
        time(&now);
        localtime_r(&now, &timeinfo);
        
        // I was getting some certificate errors, so I am pulling up
        // the system time manually if SNTP sync fails.
        if (timeinfo.tm_year < (2020 - 1900)) {
            ESP_LOGW(TAG, "SNTP sync failed (year=%d) - setting time manually to 2025-12-04",
                     timeinfo.tm_year + 1900);
            
            struct tm manual_time = {
                .tm_year = 2025 - 1900,
                .tm_mon = 12 - 1,
                .tm_mday = 4,
                .tm_hour = 12,
                .tm_min = 0,
                .tm_sec = 0
            };
            time_t t = mktime(&manual_time);
            struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            
            time(&now);
            localtime_r(&now, &timeinfo);
            ESP_LOGI(TAG, "Time manually set to: %04d-%02d-%02d %02d:%02d:%02d",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else {
            char strftime_buf[64];
            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
            ESP_LOGI(TAG, "System time synchronized via SNTP: %s", strftime_buf);
        }
    }

    g_openai_mutex = xSemaphoreCreateMutex();
    if (!g_openai_mutex) {
        ESP_LOGW("OpenAI", "Failed to create mutex; OpenAI requests may be skipped");
    }

    BaseType_t ok = xTaskCreate(
        openai_task,
        "openai_task",
        8192,
        NULL,
        4,
        &g_openai_task_handle
    );
    if (ok != pdPASS) {
        ESP_LOGW("OpenAI", "Failed to create openai_task");
        g_openai_task_handle = NULL;
    }
}