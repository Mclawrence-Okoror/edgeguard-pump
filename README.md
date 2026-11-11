# EdgeGuard Pump

**An ESP32-S3 water-level monitoring and pump-control simulation built with ESP-IDF and C.**

EdgeGuard explores how embedded systems can monitor tank water levels, automate pump-control decisions, and detect abnormal operating conditions.

The project was developed in **Wokwi** using the ESP32-S3 and ESP-IDF framework. It combines ultrasonic sensing, a simulated pump controller, an OLED dashboard, visual and audible alerts, and basic fault detection.

> **Project status:** Simulation prototype. The pump is represented by software state; it is not connected to a physical motor. Physical hardware validation has not been performed.

## Features

- Ultrasonic water-level measurement using an HC-SR04 sensor.
- Calibrated water-level estimation from sensor distance.
- Automatic simulated pump control based on configurable water-level thresholds.
- OLED dashboard showing system status.
- Green and red status LEDs.
- Audible alarms with a mute button.
- Invalid sensor-reading handling.
- Pump-stall detection based on a lack of measured water-level progress.
- FreeRTOS task for button handling.

## System Design

The ultrasonic sensor measures the distance from the sensor to the water surface. The firmware converts this distance into an estimated tank level and uses that level to determine the simulated pump state.

The OLED displays the current measurements and system status. LEDs and the buzzer provide additional feedback when the system enters an abnormal condition.

The stall-detection logic checks whether the measured water level rises while the pump is running.

## Water-Level Calibration

The current calibration uses:

- **Empty tank reference:** 50 cm
- **Full tank reference:** 5 cm
- **Pump start threshold:** 30%
- **Pump stop threshold:** 90%

The estimated water level is calculated as:

`level (%) = ((50 - distance_cm) / (50 - 5)) × 100`

The configured stop threshold leaves a margin below the calibrated full level.

## Wiring

| Component | ESP32-S3 GPIO |
|---|---|
| HC-SR04 TRIG | GPIO 5 |
| HC-SR04 ECHO | GPIO 18 |
| Green LED | GPIO 20 |
| Red LED | GPIO 19 |
| SSD1306 OLED SDA | GPIO 8 |
| SSD1306 OLED SCL | GPIO 9 |
| Buzzer | GPIO 21 |
| Mute button | GPIO 10 |

The OLED uses I²C address `0x3C`. The mute button is wired between GPIO 10 and GND, using an internal pull-up.

Refer to `diagram.json` for the Wokwi circuit configuration.

## Running the Simulation

1. Open the project in Wokwi.
2. Select the ESP32-S3 and ESP-IDF environment.
3. Ensure `main.c` is included in the application's `main` component.
4. Recreate the circuit using the supplied `diagram.json`.
5. Start the simulation and adjust the ultrasonic sensor's distance to simulate changes in water level.

**Simulation note:** Wokwi does not automatically raise the simulated water level when the software pump turns on. Change the sensor distance manually to simulate filling. If the measured level remains unchanged, the stall-detection logic may trigger.

## Testing and Validation

The following behaviours were tested in the Wokwi ESP32-S3 simulation:

- [x] Pump starts when the estimated water level reaches or falls below the configured 30% start threshold.
- [x] Calculated water level increases as the measured ultrasonic sensor distance decreases.
- [x] Pump stops when the estimated water level reaches or exceeds the configured 90% stop threshold.
- [x] Invalid ultrasonic sensor readings trigger the configured safety response and prevent pump operation.
- [x] Failure to detect sufficient water-level progress triggers the pump-stall protection mechanism.
- [x] Muting the buzzer silences audible alerts without suppressing visual fault indicators.
- [x] The OLED display and status LEDs reflect the current operating and fault states.

**Validation environment:** Wokwi ESP32-S3 simulation using ESP-IDF and FreeRTOS.

**Validation limitation:** These tests were conducted in simulation. Physical sensor accuracy, motor operation, electrical behaviour and real-world reliability have not been validated.

## Demonstration

**Live Wokwi simulation:** https://youtu.be/F21UmB2ZjOs?si=JBa_v-ZQjGuv7U8F

## Limitations and Future Work

- The project is currently a simulation, not a deployed water-management product.
- Pump operation is represented by software state; no physical motor driver or pump has been validated.
- Test results are currently based on simulation rather than physical experiments.
- Future work may include physical hardware validation, structured reliability testing, and investigation of edge-AI methods for fault detection.

## Technologies

- C
- ESP-IDF
- FreeRTOS
- ESP32-S3
- Wokwi
- HC-SR04 ultrasonic sensor
- SSD1306 OLED display
