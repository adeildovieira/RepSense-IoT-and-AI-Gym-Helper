#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/**
 * @file pin_config.h
 * @brief Pin configuration definitions for ESP32-S3-BOX (ESP32-S3-WROOM-1)
 *
 * Keep these in sync with the wiring used in main.c (IMU on I2C0, user button,
 * LEDs, and optional buzzer/temp sensors). Update if your hardware differs.
 */

// LED PINS ------------------------------------------------------------------
#define LED_PIN0 38 // Primary LED pin
#define LED_PIN1 39 // Secondary LED pin

// BUTTON PINS ---------------------------------------------------------------
// Matches BUTTON_GPIO in main.c
#define BUTTON_PIN0 10 // User button (GPIO10)

// BUZZER PINS ---------------------------------------------------------------
#define BUZZER_PIN0 21 // Ensure not in use by LCD/audio on your variant

// TEMPERATURE SENSOR PINS ---------------------------------------------------
#define ANALOG_TEMP_PIN 9   // ADC-capable
#define DIGITAL_TEMP_PIN 13 // GPIO13 for 1-wire / digital sensors

// IMU (MPU6050) I2C PINS ----------------------------------------------------
// Must stay aligned with I2C_MASTER_SDA_IO/SCL_IO in main.c (41/40)
#define MPU6050_SDA_PIN 41
#define MPU6050_SCL_PIN 40

// Convenience alias for I2C port index used by IMU
#define IMU_I2C_PORT 0

// Validate that pins are distinct and not strapping pins before deploying on
// new hardware revisions.

#endif // PIN_CONFIG_H