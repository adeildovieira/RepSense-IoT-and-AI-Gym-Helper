// main.c – RepSense v0.2 (HW button version)
// ESP32-S3-BOX-3 + Grove IMU 9DoF (ICM-20600)
// Physical button toggles START / STOP. Touchscreen not used.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

// ---- BSP / LVGL (from your lab 4) ----
#include "bsp/esp-bsp.h"
#include "lvgl.h"

static const char *TAG = "RepSense";

// ---------- I2C / IMU DEFINES ----------

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   100

// Grove IMU 9DoF default address (AD0=1 -> 0x69). Change to 0x68 if needed.
#define ICM20600_ADDR           0x69

// ICM-20600 register map (from datasheet)
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_CONFIG2       0x1D
#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75
#define REG_ACCEL_XOUT_H        0x3B

// accel scale: ACCEL_CONFIG = 0x08 -> ±4g -> 8192 LSB/g
#define ACCEL_LSB_PER_G_4G      8192.0f

// ---------- BUTTON GPIO (physical button) ----------

// CHANGE THIS to the GPIO your button is connected to.
// Very commonly the BOOT button is GPIO0, but confirm with your board.
#define BUTTON_GPIO             10

// ---------- REP / ROM PARAMETERS ----------

#define SAMPLE_RATE_HZ                 100
#define SAMPLE_PERIOD_MS               (1000 / SAMPLE_RATE_HZ)

#define CALIB_SAMPLES                  100
#define MOTION_START_THRESH_DEG        3.0f
#define DIRECTION_CHANGE_THRESH_DEG    2.0f
#define ROM_MIN_ANGLE_DEG              5.0f

#define MAX_SESSION_SECONDS            60
#define MAX_SAMPLES   (SAMPLE_RATE_HZ * MAX_SESSION_SECONDS)

// ---------- GLOBAL SESSION STATE ----------

typedef struct {
    uint32_t t_ms;
    float angle_deg;
    float ax_g, ay_g, az_g;
} sample_t;

static sample_t samples[MAX_SAMPLES];
static size_t   sample_count = 0;

static volatile bool     session_running = false;
static volatile bool     session_just_started = false;
static volatile int64_t  session_start_us = 0;

static volatile bool  baseline_ready = false;
static float          baseline_angle_deg = 0.0f;

static int            rep_count = 0;
static float          total_rom_deg = 0.0f;
static float          min_rom_deg = 0.0f;
static float          max_rom_deg = 0.0f;

// ---------- REP FSM ----------

typedef enum {
    REP_IDLE = 0,
    REP_GOING_DOWN,
    REP_GOING_UP
} rep_phase_t;

static rep_phase_t rep_phase = REP_IDLE;
static float       cur_min_angle = 0.0f;
static float       cur_max_angle = 0.0f;

// ---------- LVGL UI OBJECTS ----------

static lv_obj_t *label_status = NULL;
static lv_obj_t *label_reps   = NULL;
static lv_obj_t *label_rom    = NULL;
static lv_obj_t *label_time   = NULL;

// forward declarations
static void start_session(void);
static void stop_session(void);

// =========================================
// I2C helper functions
// =========================================

static esp_err_t icm20600_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      ICM20600_ADDR,
                                      buf, sizeof(buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t icm20600_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        ICM20600_ADDR,
                                        &reg, 1,
                                        data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// =========================================
// ICM-20600 initialization
// =========================================

static esp_err_t icm20600_init(void)
{
    esp_err_t ret;
    uint8_t who_am_i = 0;

    ret = icm20600_read_bytes(REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ICM-20600 WHO_AM_I = 0x%02X", who_am_i);
    if (who_am_i != 0x11) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I (expected 0x11), check wiring / address");
    }

    // Wake up, use PLL
    ESP_ERROR_CHECK(icm20600_write_reg(REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Gyro ±500 dps
    ESP_ERROR_CHECK(icm20600_write_reg(REG_GYRO_CONFIG, 0x08));

    // Accel ±4 g
    ESP_ERROR_CHECK(icm20600_write_reg(REG_ACCEL_CONFIG, 0x08));

    // Accel DLPF config
    ESP_ERROR_CHECK(icm20600_write_reg(REG_ACCEL_CONFIG2, 0x0B));

    // Sample rate 100 Hz: 1kHz / (1+9)
    ESP_ERROR_CHECK(icm20600_write_reg(REG_SMPLRT_DIV, 9));

    // CONFIG: DLPF 3
    ESP_ERROR_CHECK(icm20600_write_reg(REG_CONFIG, 0x03));

    ESP_LOGI(TAG, "ICM-20600 initialized");
    return ESP_OK;
}

// =========================================
// Read accelerometer data (g units)
// =========================================

static esp_err_t icm20600_read_accel(float *ax_g, float *ay_g, float *az_g)
{
    uint8_t raw[6];
    esp_err_t ret = icm20600_read_bytes(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    *ax_g = (float)ax / ACCEL_LSB_PER_G_4G;
    *ay_g = (float)ay / ACCEL_LSB_PER_G_4G;
    *az_g = (float)az / ACCEL_LSB_PER_G_4G;

    return ESP_OK;
}

// =========================================
// Angle computation
// =========================================

static float compute_angle_deg(float ax_g, float ay_g, float az_g)
{
    float denom = sqrtf(ay_g * ay_g + az_g * az_g);
    if (denom < 1e-6f) denom = 1e-6f;
    float angle_rad = atanf(ax_g / denom);
    return angle_rad * 180.0f / (float)M_PI;
}

// =========================================
// REP / ROM state management
// =========================================

static void reset_rep_state(void)
{
    rep_phase = REP_IDLE;
    cur_min_angle = 0.0f;
    cur_max_angle = 0.0f;

    rep_count = 0;
    total_rom_deg = 0.0f;
    min_rom_deg = 0.0f;
    max_rom_deg = 0.0f;
}

static void rep_fsm_update(float angle_deg)
{
    switch (rep_phase) {
    case REP_IDLE:
        cur_min_angle = cur_max_angle = angle_deg;
        if (baseline_ready &&
            fabsf(angle_deg - baseline_angle_deg) > MOTION_START_THRESH_DEG) {
            if (angle_deg < baseline_angle_deg) {
                rep_phase = REP_GOING_DOWN;
            } else {
                rep_phase = REP_GOING_UP;
            }
        }
        break;

    case REP_GOING_DOWN:
        if (angle_deg < cur_min_angle) cur_min_angle = angle_deg;
        if (angle_deg > cur_min_angle + DIRECTION_CHANGE_THRESH_DEG) {
            rep_phase = REP_GOING_UP;
            cur_max_angle = angle_deg;
        }
        break;

    case REP_GOING_UP:
        if (angle_deg > cur_max_angle) cur_max_angle = angle_deg;
        if (angle_deg < cur_max_angle - DIRECTION_CHANGE_THRESH_DEG) {
            float rom = fabsf(cur_max_angle - cur_min_angle);
            if (rom >= ROM_MIN_ANGLE_DEG) {
                rep_count++;
                total_rom_deg += rom;
                if (min_rom_deg == 0.0f || rom < min_rom_deg) min_rom_deg = rom;
                if (rom > max_rom_deg) max_rom_deg = rom;
                ESP_LOGI(TAG, "Rep %d, ROM = %.2f deg", rep_count, rom);
            }
            rep_phase = REP_GOING_DOWN;
            cur_min_angle = cur_max_angle = angle_deg;
        }
        break;
    }
}

// =========================================
// LVGL UI helpers
// =========================================

static void update_labels_running(void)
{
    if (!label_status || !label_reps || !label_rom || !label_time) return;

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = now_us - session_start_us;
    if (dt_us < 0) dt_us = 0;
    float seconds = (float)dt_us / 1e6f;

    char buf[96];

    snprintf(buf, sizeof(buf), "RepSense: RUNNING (press button to STOP)");
    lv_label_set_text(label_status, buf);

    snprintf(buf, sizeof(buf), "Time: %.1f s", seconds);
    lv_label_set_text(label_time, buf);

    snprintf(buf, sizeof(buf), "Reps: %d", rep_count);
    lv_label_set_text(label_reps, buf);

    if (rep_count > 0) {
        float avg_rom = total_rom_deg / (float)rep_count;
        snprintf(buf, sizeof(buf),
                 "ROM avg: %.1f deg\nmin: %.1f max: %.1f",
                 avg_rom, min_rom_deg, max_rom_deg);
    } else {
        snprintf(buf, sizeof(buf),
                 "ROM avg: 0.0 deg\nmin: 0.0 max: 0.0");
    }
    lv_label_set_text(label_rom, buf);
}

static void update_labels_idle(const char *reason)
{
    if (!label_status || !label_reps || !label_rom || !label_time) return;

    char buf[96];

    snprintf(buf, sizeof(buf), "RepSense: %s\nPress button to START", reason ? reason : "IDLE");
    lv_label_set_text(label_status, buf);

    lv_label_set_text(label_time, "Time: 0.0 s");
    lv_label_set_text(label_reps, "Reps: 0");
    lv_label_set_text(label_rom,
                      "ROM avg: 0.0 deg\nmin: 0.0 max: 0.0");
}

static void ui_timer_cb(lv_timer_t *timer)
{
    (void) timer;
    if (session_running) {
        update_labels_running();
    }
}

// Simple screen with labels only
static void create_main_screen(void)
{
    lv_obj_t *scr = lv_scr_act();

    label_status = lv_label_create(scr);
    lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_long_mode(label_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_status, 220);

    label_time = lv_label_create(scr);
    lv_obj_align(label_time, LV_ALIGN_TOP_MID, 0, 50);

    label_reps = lv_label_create(scr);
    lv_obj_align(label_reps, LV_ALIGN_TOP_MID, 0, 80);

    label_rom = lv_label_create(scr);
    lv_label_set_long_mode(label_rom, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_rom, 220);
    lv_obj_align(label_rom, LV_ALIGN_TOP_MID, 0, 110);

    update_labels_idle("READY");

    lv_timer_create(ui_timer_cb, 200, NULL);
}

// =========================================
// Session control
// =========================================

static void start_session(void)
{
    if (session_running) return;

    ESP_LOGI(TAG, "Session START");

    sample_count = 0;
    baseline_ready = false;
    baseline_angle_deg = 0.0f;
    reset_rep_state();

    session_start_us = esp_timer_get_time();
    session_running = true;
    session_just_started = true;

    update_labels_idle("CALIBRATING...");
}

static void stop_session(void)
{
    if (!session_running) return;

    session_running = false;

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = now_us - session_start_us;
    if (dt_us < 0) dt_us = 0;
    float seconds = (float)dt_us / 1e6f;

    ESP_LOGI(TAG, "Session STOP: time = %.2f s, reps = %d", seconds, rep_count);

    if (label_status && label_reps && label_rom && label_time) {
        char buf[96];

        snprintf(buf, sizeof(buf), "RepSense: DONE\nPress button to START again");
        lv_label_set_text(label_status, buf);

        snprintf(buf, sizeof(buf), "Time: %.1f s", seconds);
        lv_label_set_text(label_time, buf);

        snprintf(buf, sizeof(buf), "Reps: %d", rep_count);
        lv_label_set_text(label_reps, buf);

        if (rep_count > 0) {
            float avg_rom = total_rom_deg / (float)rep_count;
            snprintf(buf, sizeof(buf),
                     "ROM avg: %.1f deg\nmin: %.1f max: %.1f",
                     avg_rom, min_rom_deg, max_rom_deg);
        } else {
            snprintf(buf, sizeof(buf),
                     "ROM avg: 0.0 deg\nmin: 0.0 max: 0.0");
        }
        lv_label_set_text(label_rom, buf);
    }
}

// =========================================
// IMU sampling task
// =========================================

static void imu_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "IMU task started");

    int calib_count = 0;
    float calib_sum = 0.0f;
    bool prev_running = false;

    while (1) {
        if (!session_running) {
            prev_running = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!prev_running) {
            calib_count = 0;
            calib_sum = 0.0f;
            baseline_ready = false;
            prev_running = true;
            ESP_LOGI(TAG, "Calibration phase started");
        }

        float ax_g, ay_g, az_g;
        esp_err_t ret = icm20600_read_accel(&ax_g, &ay_g, &az_g);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read accel: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        float angle_deg = compute_angle_deg(ax_g, ay_g, az_g);

        if (!baseline_ready) {
            calib_sum += angle_deg;
            calib_count++;
            if (calib_count >= CALIB_SAMPLES) {
                baseline_angle_deg = calib_sum / (float)calib_count;
                baseline_ready = true;
                ESP_LOGI(TAG, "Baseline angle = %.2f deg", baseline_angle_deg);
            }
        }

        if (sample_count < MAX_SAMPLES) {
            int64_t now_us = esp_timer_get_time();
            uint32_t t_ms = (uint32_t)((now_us - session_start_us) / 1000);
            samples[sample_count].t_ms = t_ms;
            samples[sample_count].angle_deg = angle_deg;
            samples[sample_count].ax_g = ax_g;
            samples[sample_count].ay_g = ay_g;
            samples[sample_count].az_g = az_g;
            sample_count++;
        }

        if (baseline_ready) {
            rep_fsm_update(angle_deg);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// =========================================
// Button task (polling with debounce)
// =========================================

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
    (void) arg;
    ESP_LOGI(TAG, "Button task started on GPIO %d", BUTTON_GPIO);

    int last_level = gpio_get_level(BUTTON_GPIO);

    while (1) {
        int level = gpio_get_level(BUTTON_GPIO);

        // detect falling edge (HIGH -> LOW)
        if (last_level == 1 && level == 0) {
            // debounce
            vTaskDelay(pdMS_TO_TICKS(30));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                // toggle session
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

// =========================================
// app_main
// =========================================

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(bsp_i2c_init());

    lv_disp_t *disp = bsp_display_start();
    (void) disp;
    bsp_display_brightness_set(80);

    bsp_display_lock(0);
    create_main_screen();
    bsp_display_unlock();

    ESP_ERROR_CHECK(icm20600_init());

    button_init();

    xTaskCreate(imu_task,    "imu_task",    4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 6, NULL);

    ESP_LOGI(TAG, "RepSense setup complete – press the button to START/STOP");
}