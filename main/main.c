#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_crt_bundle.h"

#include "esp32s3_box_lcd_config.h"

#define WIFI_SSID "iPhone"
#define WIFI_PASS "senhadoade"

#define LATITUDE  "36.00"
#define LONGITUDE "-78.90"
#define API_KEY   "9796d9841f8da1583904e5bfa8ba6f0c"

#define WEATHER_API_URL \
  "https://api.openweathermap.org/data/2.5/weather?lat=" LATITUDE \
  "&lon=" LONGITUDE "&units=metric&appid=" API_KEY

#ifndef CONFIG_ESP_MAXIMUM_RETRY
#define CONFIG_ESP_MAXIMUM_RETRY 5
#endif

static const char *TAG = "lab4_task4";

static lv_disp_t *disp;
static lv_obj_t *temp_label, *humd_label, *wind_label;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static lv_disp_t *gui_setup(void) {
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

  gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
                                  .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
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
      .rotation = {.swap_xy = false, .mirror_x = true, .mirror_y = true}};
  return lvgl_port_add_disp(&disp_cfg);
}

static void update_display(const char *temp, const char *humd, const char *wind) {
  if (!lvgl_port_lock(5000)) return;
  lv_obj_t *scr = lv_disp_get_scr_act(disp);

  lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  if (!temp_label) {
    temp_label = lv_label_create(scr);
    humd_label = lv_label_create(scr);
    wind_label = lv_label_create(scr);
    lv_obj_set_style_text_color(temp_label, lv_color_black(), 0);
    lv_obj_set_style_text_color(humd_label, lv_color_black(), 0);
    lv_obj_set_style_text_color(wind_label, lv_color_black(), 0);
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 20, 40);
    lv_obj_align(humd_label, LV_ALIGN_TOP_LEFT, 20, 80);
    lv_obj_align(wind_label, LV_ALIGN_TOP_LEFT, 20, 120);
  }
  lv_label_set_text(temp_label, temp);
  lv_label_set_text(humd_label, humd);
  lv_label_set_text(wind_label, wind);
  lvgl_port_unlock();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retrying Wi-Fi (%d/%d)", s_retry_num, CONFIG_ESP_MAXIMUM_RETRY);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static esp_err_t wifi_connect(void) {
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PASS,
          .threshold.authmode = (strlen(WIFI_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);
  vEventGroupDelete(s_wifi_event_group);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to Wi-Fi SSID:%s", WIFI_SSID);
    return ESP_OK;
  }
  ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
  return ESP_FAIL;
}

//http
static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
  static char *output_buffer = NULL;
  static int output_len = 0;
  static int alloc_len = 0;

  switch (evt->event_id) {
  case HTTP_EVENT_ON_DATA: {
    if (!output_buffer) {
      int content_length = esp_http_client_get_content_length(evt->client);
      if (content_length <= 0) content_length = 4096;
      alloc_len = content_length + 1;
      output_buffer = (char *)malloc(alloc_len);
      output_len = 0;
      if (!output_buffer) return ESP_FAIL;
    }
    if (output_len + evt->data_len + 1 > alloc_len) {
      alloc_len = output_len + evt->data_len + 1024;
      char *nbuf = (char *)realloc(output_buffer, alloc_len);
      if (!nbuf) { free(output_buffer); output_buffer = NULL; return ESP_FAIL; }
      output_buffer = nbuf;
    }
    memcpy(output_buffer + output_len, evt->data, evt->data_len);
    output_len += evt->data_len;
    output_buffer[output_len] = '\0';
    break;
  }
  case HTTP_EVENT_ON_FINISH: {
    if (output_buffer) {
      char **resp_ptr = (char **)evt->user_data;
      *resp_ptr = output_buffer;  // hand off
      output_buffer = NULL;
      output_len = 0;
      alloc_len = 0;
    }
    break;
  }
  case HTTP_EVENT_DISCONNECTED: {
    if (output_buffer) { free(output_buffer); output_buffer = NULL; }
    output_len = 0; alloc_len = 0;
    break;
  }
  default:
    break;
  }
  return ESP_OK;
}

static void fetch_and_parse_weather(void) {
  if (API_KEY[0] == '\0') { update_display("Error:", "Missing API Key", ""); return; }

  ESP_LOGI(TAG, "Fetching: %s", WEATHER_API_URL);

  char *response_buffer = NULL;
  esp_http_client_config_t config = {
      .url = WEATHER_API_URL,
      .event_handler = _http_event_handler,
      .user_data = &response_buffer,
      .disable_auto_redirect = false,
      .crt_bundle_attach = esp_crt_bundle_attach,
      .max_authorization_retries = -1,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_err_t err = esp_http_client_perform(client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP GET failed: %s", esp_err_to_name(err));
    update_display("Error:", "HTTP Request Fail", "");
    goto cleanup;
  }

  int status = esp_http_client_get_status_code(client);
  if (status != 200) {
    char preview[128] = {0};
    if (response_buffer) {
      size_t n = strlen(response_buffer);
      if (n > sizeof(preview)-1) n = sizeof(preview)-1;
      memcpy(preview, response_buffer, n);
    }
    ESP_LOGE(TAG, "HTTP %d. Body: %s", status, preview);
    char msg[32]; snprintf(msg, sizeof(msg), "HTTP %d", status);
    update_display("Error:", msg, "");
    goto cleanup;
  }

  if (!response_buffer) { update_display("Error:", "No body", ""); goto cleanup; }

  // main:{temp,humidity}, wind:{speed}
  cJSON *root = cJSON_Parse(response_buffer);
  if (!root) { update_display("Error:", "JSON Parse Fail", ""); goto cleanup; }

  cJSON *main = cJSON_GetObjectItemCaseSensitive(root, "main");
  cJSON *wind = cJSON_GetObjectItemCaseSensitive(root, "wind");
  cJSON *temp = main ? cJSON_GetObjectItemCaseSensitive(main, "temp") : NULL;
  cJSON *humidity = main ? cJSON_GetObjectItemCaseSensitive(main, "humidity") : NULL;
  cJSON *wind_speed = wind ? cJSON_GetObjectItemCaseSensitive(wind, "speed") : NULL;

  char temp_str[32], humd_str[32], wind_str[32];

  if (cJSON_IsNumber(temp)) {
    double temp_c = temp->valuedouble;
    double temp_f = temp_c * 1.8 + 32.0;
    snprintf(temp_str, sizeof(temp_str), "Temp: %.1f F", temp_f);
  } else snprintf(temp_str, sizeof(temp_str), "Temp: N/A");

  if (cJSON_IsNumber(humidity)) {
    snprintf(humd_str, sizeof(humd_str), "Humd: %.1f %%", humidity->valuedouble);
  } else snprintf(humd_str, sizeof(humd_str), "Humd: N/A");

  if (cJSON_IsNumber(wind_speed)) {
    snprintf(wind_str, sizeof(wind_str), "Wind: %.1f m/s", wind_speed->valuedouble);
  } else snprintf(wind_str, sizeof(wind_str), "Wind: N/A");

  update_display(temp_str, humd_str, wind_str);
  cJSON_Delete(root);

cleanup:
  if (response_buffer) free(response_buffer);
  esp_http_client_cleanup(client);
}


static void main_task(void *pv) {
  update_display("Connecting to Wi-Fi...", "", "");
  if (wifi_connect() == ESP_OK) {
    update_display("Fetching Weather...", "", "");
    fetch_and_parse_weather();
  } else {
    update_display("Wi-Fi Failed", "Please restart", "");
  }
  vTaskDelete(NULL);
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  disp = gui_setup();
  xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}