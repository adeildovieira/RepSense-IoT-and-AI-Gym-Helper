# RepSense - IoT Gym Rep Counter with AI Coaching

## Overview

RepSense is an embedded IoT system for bench press rep counting and form analysis. The system uses an ESP32-S3-BOX-3 microcontroller with a 9-axis IMU sensor to track workout performance and provides real-time feedback through an LCD display. Post-workout session data is sent to OpenAI's GPT-4o-mini model for coaching feedback.

## Hardware Requirements

- ESP32-S3-BOX-3 development board
- Grove IMU 9DOF (ICM-20600) 6-axis accelerometer/gyroscope
- ILI9341 320x240 LCD display (integrated on ESP32-S3-BOX-3)
- GPIO button for session control

## Software Requirements

- ESP-IDF v5.x
- FreeRTOS (included with ESP-IDF)
- LVGL graphics library
- mbedTLS for HTTPS communication
- OpenAI API key

## System Architecture

### Core Components

1. **IMU Sampling Task**: Reads accelerometer data at 100 Hz
2. **Rep Detection Algorithm**: Gravity-relative vertical acceleration analysis
3. **Session Management**: Start/stop workout sessions via button press
4. **Display Interface**: Real-time rep count and session time display
5. **Network Module**: WiFi connection and HTTPS communication
6. **OpenAI Integration**: Post-session data analysis and coaching feedback

### Physics-Based Rep Detection

The system extracts vertical acceleration relative to gravity, making rep detection independent of device orientation or user arm length. Key features:

- 1-second calibration phase to establish baseline gravity vector
- Motion detection threshold: 0.02g
- Direction change tracking for up/down motion phases
- Minimum peak acceleration requirement: 0.075g
- Pitch angle calculation for left/right imbalance detection

## Data Format

Session data is transmitted using a custom text format called RepSenseToon v1:

```
RepSenseToon v1
session_id: [number]
time_s: [duration]
reps: [count]
imbalance_deg: [angle]
```

## Configuration

### WiFi Settings

Edit `main/main.c`:

```c
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourPassword"
```

### OpenAI API

Set your API key in `main/main.c`:

```c
#define OPENAI_API_KEY "your-api-key-here"
```

The system uses the `/v1/chat/completions` endpoint with the `gpt-4o-mini` model.

### IMU Configuration

Default I2C settings:

```c
#define I2C_MASTER_SCL_IO 40
#define I2C_MASTER_SDA_IO 41
#define I2C_MASTER_FREQ_HZ 400000
#define IMU_ADDR_DEFAULT 0x68
```

## Build and Flash

```bash
idf.py build
idf.py flash
idf.py monitor
```

## Usage

1. Power on the device
2. Wait for WiFi connection and IMU initialization
3. Press the button to start a workout session
4. Perform bench press reps (device should be attached to the bar or weight)
5. Press the button again to stop the session
6. Session metrics are displayed on the LCD
7. AI coaching feedback is requested in the background

## Session Metrics

- **Total Time**: Duration of the workout session
- **Rep Count**: Number of valid repetitions detected
- **Imbalance**: Angular difference between start and end positions (degrees)

## AI Coaching Prompt

The system sends session data with the following instructions:

- Assess the set (fatigue, pace, stability)
- Comment on imbalance in simple terms (left/right balance)
- Suggest one practical tip for the next set
- Response limited to 3 bullet points

## Detection Parameters

All thresholds can be adjusted in `main/main.c`:

```c
#define SAMPLE_RATE_HZ 100
#define CALIB_SAMPLES 100
#define VERT_SIGN_DEADZONE_G 0.02f
#define VERT_MOTION_THRESH_G 0.02f
#define VERT_RETURN_THRESH_G 0.02f
#define VERT_PEAK_MIN_G 0.075f
```

## Network Configuration

- Uses DukeOpen WiFi (open network configuration)
- SNTP time synchronization with fallback to manual time setting
- HTTPS with TLS certificate bundle validation
- Content-Length header explicitly set for OpenAI API compatibility

## Known Issues

- SNTP may fail on restricted networks (manual time fallback implemented)
- Some network configurations may require adjusting SSL certificate validation settings

## Project Structure

```
RepSense-IoT-and-AI-Gym-Helper/
├── main/
│   ├── main.c                    # Main application code
│   ├── CMakeLists.txt
│   ├── component.mk
│   ├── esp32s3_box_lcd_config.h  # LCD configuration
│   ├── pin_config.h              # Pin definitions
│   ├── idf_component.yml
│   └── Kconfig.projbuild
├── build/                        # Build artifacts
├── CMakeLists.txt
├── Makefile
├── sdkconfig                     # ESP-IDF configuration
└── README.md
```

## License

See LICENSE file for details. Base files including main.c are modified from Lab 4 of ECE 655 from Duke University.