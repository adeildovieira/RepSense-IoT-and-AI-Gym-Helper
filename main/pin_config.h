#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/**
 * @file pin_config.h
 * @brief Pin configuration definitions for ESP32-S3 board
 *
 * This header file defines the pin layout for the ESP32-S3 development board.
 * Modify these definitions based on your specific hardware configuration.
 */

// ============================================================================
// LED PINS
// ============================================================================
#define LED_PIN0 38 // Primary LED pin
#define LED_PIN1 39 // Secondary LED pin

// ============================================================================
// BUTTON PINS
// ============================================================================
#define BUTTON_PIN0 40 // Primary button pin

// ============================================================================
// BUZZER PINS
// ============================================================================
#define BUZZER_PIN0 21

// ============================================================================
// TEMPERATURE PINS
// ============================================================================
#define ANALOG_TEMP_PIN 9
#define DIGITAL_TEMP_PIN 13

// ============================================================================
// MPU6050 PINS
// ============================================================================
#define MPU6050_SDA_PIN 10
#define MPU6050_SCL_PIN 11

#endif // PIN_CONFIG_H