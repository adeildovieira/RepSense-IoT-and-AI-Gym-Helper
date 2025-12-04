// main.c – RepSense Lite
// ESP32-S3-BOX-3 + Grove IMU 9DOF (0x68)
// Bench press rep counter using vertical acceleration (relative to gravity)
//
// - Display init matches Lab 4 (esp32s3_box_lcd_config.h).
// - I2C IMU on SCL=GPIO40, SDA=GPIO41.
// - Button on GPIO10 toggles START/STOP.
// - Calibrates gravity direction, uses vertical accel for reps.
// - Minimal UI: status, time, reps (no ROM / no sample buffer).

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

#include "esp32s3_box_lcd_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "RepSense";

// -------------------- I2C / IMU DEFINES --------------------

#define I2C_MASTER_SCL_IO          40
#define I2C_MASTER_SDA_IO          41
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TIMEOUT_MS      100

#define IMU_ADDR_DEFAULT           0x68

// IMU registers (ICM/MPU family)
#define REG_SMPLRT_DIV             0x19
#define REG_CONFIG                 0x1A
#define REG_GYRO_CONFIG            0x1B
#define REG_ACCEL_CONFIG           0x1C
#define REG_ACCEL_CONFIG2          0x1D
#define REG_PWR_MGMT_1             0x6B
#define REG_WHO_AM_I               0x75
#define REG_ACCEL_XOUT_H           0x3B

// Accel scale: assume ±4 g -> 8192 LSB/g
#define ACCEL_LSB_PER_G_4G         8192.0f

// -------------------- BUTTON GPIO --------------------

#define BUTTON_GPIO                10

// -------------------- SAMPLING / SESSION PARAMS --------------------

#define SAMPLE_RATE_HZ             100
#define SAMPLE_PERIOD_MS           (1000 / SAMPLE_RATE_HZ)

#define CALIB_SAMPLES              100   // ~1 s of calibration at 100 Hz

// Vertical-accel based rep detection (bench press)
#define VERT_SIGN_DEADZONE_G       0.02f   // ignore tiny noise
#define VERT_MOTION_THRESH_G       0.02f   // start rep if |a_vert| >= this
#define VERT_RETURN_THRESH_G       0.02f   // "back to baseline" if |a_vert| <= this

// New: rep must reach at least this peak vertical accel to be valid
#define VERT_PEAK_MIN_G            0.075f   // require >= 0.075 g at some point in the rep

// If you really want a ROM threshold, we’d need angle; here we focus on reps only.

// -------------------- GLOBAL SESSION STATE --------------------

static volatile bool     session_running   = false;
static volatile int64_t  session_start_us  = 0;

// Baseline gravity
static volatile bool  baseline_ready       = false;
static float          baseline_acc_g       = 0.0f;   // |g0|
static float          g_unit_x             = 0.0f;
static float          g_unit_y             = 0.0f;
static float          g_unit_z             = 0.0f;

// Rep statistics
static int   rep_count     = 0;

// Rep detection state
static bool  rep_in_motion    = false;
static int   direction_changes = 0;
static int   last_sign         = 0;

static float rep_peak_abs_vert = 0.0f;

// IMU I2C address
static uint8_t g_imu_addr = IMU_ADDR_DEFAULT;

// -------------------- LVGL OBJECTS --------------------

static lv_disp_t *disp         = NULL;
static lv_obj_t  *label_status = NULL;
static lv_obj_t  *label_time   = NULL;
static lv_obj_t  *label_reps   = NULL;

// ----------------------------------------------------
// I2C INIT
// ----------------------------------------------------

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

// ----------------------------------------------------
// IMU helpers
// ----------------------------------------------------

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

    // Wake up, use PLL
    ret = imu_write_reg(REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Gyro ±500 dps
    ret = imu_write_reg(REG_GYRO_CONFIG, 0x08);
    if (ret != ESP_OK) goto fail;

    // Accel ±4 g
    ret = imu_write_reg(REG_ACCEL_CONFIG, 0x08);
    if (ret != ESP_OK) goto fail;

    // Accel DLPF config
    ret = imu_write_reg(REG_ACCEL_CONFIG2, 0x0B);
    if (ret != ESP_OK) goto fail;

    // Sample rate 100 Hz: 1kHz / (1+9)
    ret = imu_write_reg(REG_SMPLRT_DIV, 9);
    if (ret != ESP_OK) goto fail;

    // CONFIG: DLPF 3
    ret = imu_write_reg(REG_CONFIG, 0x03);
    if (ret != ESP_OK) goto fail;

    ESP_LOGI(TAG, "IMU initialized successfully at 0x%02X", g_imu_addr);
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

// ----------------------------------------------------
// LVGL UI helpers
// ----------------------------------------------------

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

static void update_labels_done(float total_time_s)
{
    if (!label_status || !label_time || !label_reps) return;

    char buf[160];
    lvgl_port_lock(0);

    snprintf(buf, sizeof(buf),
             "RepSense: DONE\nPress button to START again");
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

// ----------------------------------------------------
// Session control
// ----------------------------------------------------

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

    baseline_ready   = false;
    baseline_acc_g   = 0.0f;
    g_unit_x = g_unit_y = g_unit_z = 0.0f;

    reset_rep_state();

    session_start_us = esp_timer_get_time();
    session_running  = true;

    update_labels_idle("CALIBRATING...");
}

static void stop_session(void)
{
    if (!session_running) return;

    session_running = false;

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us  = now_us - session_start_us;
    if (dt_us < 0) dt_us = 0;
    float seconds = (float)dt_us / 1e6f;

    ESP_LOGI(TAG, "Session STOP: time = %.2f s, reps = %d", seconds, rep_count);

    update_labels_done(seconds);
}

// ----------------------------------------------------
// Rep detection using vertical acceleration (bench press)
// ----------------------------------------------------

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

        // Direction change: sign flip of vertical accel
        if (sign != 0 && last_sign != 0 && sign != last_sign) {
            direction_changes++;
            ESP_LOGI(TAG, "Direction change %d (sign %d -> %d, a_vert=%.3f)",
                     direction_changes, last_sign, sign, a_vert);
            last_sign = sign;
        }

        // If we've seen at least 2 direction changes and come back near baseline,
        // we treat that as a *candidate* rep.
        if (direction_changes >= 2 && abs_vert <= VERT_RETURN_THRESH_G) {
            if (rep_peak_abs_vert >= VERT_PEAK_MIN_G) {
                // Valid rep: sufficiently strong
                rep_count++;
                ESP_LOGI(TAG, "Rep %d (peak |a_vert|=%.3f g)", rep_count, rep_peak_abs_vert);
            } else {
                // Too weak: likely noise
                ESP_LOGI(TAG,
                         "Motion ended but peak |a_vert|=%.3f g < %.3f g (not counting)",
                         rep_peak_abs_vert, VERT_PEAK_MIN_G);
            }

            // Reset motion state
            rep_in_motion      = false;
            direction_changes  = 0;
            last_sign          = 0;
            rep_peak_abs_vert  = 0.0f;
        }
    }
}

// ----------------------------------------------------
// IMU sampling task
// ----------------------------------------------------

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
            // Collect samples for baseline
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

                // gravity unit vector (down direction)
                g_unit_x = avg_ax / baseline_acc_g;
                g_unit_y = avg_ay / baseline_acc_g;
                g_unit_z = avg_az / baseline_acc_g;

                baseline_ready = true;
                ESP_LOGI(TAG,
                         "Baseline |acc| = %.3f g, g_unit=(%.3f, %.3f, %.3f)",
                         baseline_acc_g, g_unit_x, g_unit_y, g_unit_z);
            }
        } else {
            // Vertical acceleration along gravity
            float proj   = ax_g * g_unit_x + ay_g * g_unit_y + az_g * g_unit_z;
            float a_vert = proj - baseline_acc_g;   // dynamic vertical accel

            rep_update_from_vertical(a_vert);

            // Debug every ~10 samples
            dbg_counter++;
            if (dbg_counter >= 10) {
                dbg_counter = 0;
                ESP_LOGI(TAG,
                         "a_vert=%.3f g, reps=%d, running=%d",
                         a_vert, rep_count, session_running ? 1 : 0);
            }
        }

        // Update UI every ~10 samples
        ui_counter++;
        if (ui_counter >= 10) {
            ui_counter = 0;
            update_labels_running();
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ----------------------------------------------------
// Button task (polling + debounce)
// ----------------------------------------------------

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

        // Detect falling edge (HIGH -> LOW)
        if (last_level == 1 && level == 0) {
            vTaskDelay(pdMS_TO_TICKS(30)); // debounce
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

// ----------------------------------------------------
// app_main
// ----------------------------------------------------

void app_main(void)
{
    // Display + LVGL
    disp = gui_setup();

    lvgl_port_lock(0);
    create_main_screen();
    lvgl_port_unlock();

    update_labels_idle("INIT I2C/IMU...");

    // I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        update_labels_idle("I2C ERROR");
        return;
    }

    // IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(ret));
        update_labels_idle("IMU ERROR");
        return;
    }

    // Button + tasks
    button_init();

    xTaskCreate(imu_task,    "imu_task",    4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 6, NULL);

    ESP_LOGI(TAG, "RepSense ready – press the button to START/STOP");
}