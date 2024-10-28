# Home Surveillance System

This project implements a home surveillance system that integrates DC motor control, stepper motor-driven locking mechanism, force sensor-based detection with a spray pump and siren, and GPS tracking capabilities, all controlled through the Blynk app using an ESP32 microcontroller.

## Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Pin Configuration](#pin-configuration)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Blynk Virtual Pins](#blynk-virtual-pins)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project uses an ESP32 microcontroller to build a remote-controlled home surveillance system. The system provides features like:
- DC motor control for motion mechanisms.
- Stepper motor control for locking/unlocking doors.
- Force sensor to detect intrusions and trigger a spray and siren system.
- GPS tracking via the Blynk app.
- Integration with Blynk for remote control and monitoring.

## Hardware Requirements

1. **ESP32 Development Board**
2. **DC Motor** (controlled via L298N Motor Driver)
3. **Stepper Motor** (controlled via L298N Motor Driver)
4. **Limit Switches** (for detecting motor boundaries)
5. **Force Sensor** (to detect pressure applied on doors)
6. **Pump** (to trigger spray mechanism)
7. **GPS Module** (connected via UART)
8. **L298N Motor Driver**
9. **Power supply**
10. **Jumper wires**

## Software Requirements

- **Arduino IDE** with ESP32 board support.
- **Blynk Library**: Download from [Blynk GitHub](https://github.com/blynkkk/blynk-library)
- **TinyGPS++ Library**: Download from [TinyGPS++ GitHub](https://github.com/mikalhart/TinyGPSPlus)
- **Stepper Library**: Built-in library in the Arduino IDE.

## Pin Configuration

The system uses the following pins for different components:

### Motor Control (L298N Driver)
- **DC Motor IN1**: GPIO 2
- **DC Motor IN2**: GPIO 4
- **DC Motor Enable**: GPIO 16

### Stepper Motor (L298N Driver)
- **Stepper IN1**: GPIO 14
- **Stepper IN2**: GPIO 12
- **Stepper IN3**: GPIO 13
- **Stepper IN4**: GPIO 15

### Limit Switches
- **Open Limit Switch**: GPIO 32
- **Close Limit Switch**: GPIO 33

### Force Sensor and Pump Control
- **Force Sensor Pin**: GPIO 34
- **Pump IN1**: GPIO 19
- **Pump IN2**: GPIO 18

### GPS Module (UART)
- **GPS RX (ESP32 TX)**: GPIO 1
- **GPS TX (ESP32 RX)**: GPIO 3

### Blynk Virtual Pins
- **VPIN_MOTOR**: V1
- **VPIN_LOCK_UNLOCK_DOOR**: V2
- **VPIN_SIREN_SPRAY**: V4
- **VPIN_GPS_LAT_LONG**: V5
- **VPIN_GPS_SWITCH**: V6

## Features

- **DC Motor Control**: Remotely control the DC motor via the Blynk app, allowing clockwise or anticlockwise rotation based on user input.
- **Door Locking Mechanism**: Use a stepper motor to lock or unlock the door, controlled via the Blynk app.
- **Force Sensor Detection**: Detect intrusions using a force sensor, which triggers a spray and siren alarm.
- **GPS Tracking**: Display GPS coordinates on the Blynk app, allowing real-time location tracking.
- **Limit Switch Integration**: Prevent motor over-rotation by using limit switches to detect boundaries.

## Installation

1. **Install Required Libraries**:
    - Install the Blynk library in Arduino IDE by navigating to `Sketch` > `Include Library` > `Manage Libraries` and searching for "Blynk".
    - Install the TinyGPS++ library by searching for "TinyGPS++" in the library manager.
    
2. **Configure Blynk**:
    - Create a new Blynk template and note down the Template ID, Device Name, and Authentication Token.
    - Update the `BLYNK_TEMPLATE_ID`, `BLYNK_TEMPLATE_NAME`, and `BLYNK_AUTH_TOKEN` in the code with your Blynk credentials.

3. **Wi-Fi Configuration**:
    - Replace the Wi-Fi SSID and password in the code with your Wi-Fi credentials:
      ```cpp
      const char *ssid = "your_wifi_ssid";
      const char *pass = "your_wifi_password";
      ```

4. **Upload Code**:
    - Connect your ESP32 board to your computer and upload the provided code using the Arduino IDE.

5. **Blynk Setup**:
    - In the Blynk app, create buttons, sliders, and labels corresponding to the virtual pins defined in the code.

## Usage

Once the project is set up and running:
1. Use the Blynk app to control the DC motor and the stepper motor for door locking and unlocking.
2. Monitor the force sensor. If force is detected, the pump and siren are activated for a preset time.
3. Use the GPS switch in the app to start or stop GPS tracking and display the location in real-time.
4. Control the spray mechanism and siren manually using the Blynk interface.

### Blynk Virtual Pins

- **V1**: Controls the DC motor (ON/OFF and direction)
- **V2**: Locks/unlocks the door using the stepper motor
- **V4**: Triggers the siren and spray manually
- **V5**: Displays the GPS location (latitude, longitude)
- **V6**: Toggles GPS tracking (ON/OFF)

## Contributing

Feel free to contribute to this project by submitting pull requests or reporting issues. Any suggestions for improvement are welcome!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.