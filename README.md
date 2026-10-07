# Soul

English | [Português](README_pt.md)

<!-- Robot photo: add docs/images/soul.jpg and uncomment.
<p align="center">
  <img src="docs/images/soul.jpg" alt="Soul autonomous rescue robot" width="600"/>
</p>
-->

## About Soul

Soul is an autonomous rescue robot designed and built in-house by ORC Robotics. On its own, it follows a line track, reads turn markers and gets past gaps, ramps and obstacles. In the rescue area, it finds the victims, collects them, delivers each one to its zone and makes its way out.

### Compact and complete

Despite its compact size, Soul carries a complete robotics stack. A **Raspberry Pi** runs the mission strategy in C++ and the computer vision in Python, while an **ESP32** drives the motors and servos and streams sensor data back over UART. Two cameras, wheel encoders, an IMU and an ultrasonic sensor let it read the track and its surroundings, and a web dashboard shows everything live.

## Key Features

### Navigation
- **Line following** — track segmentation from the down-facing camera, with front-camera assistance
- **Turn markers** — green markers matched to the line and confirmed over time, with IMU-guided turns
- **Gaps and line recovery** — continuity checks and front-camera reacquisition
- **Obstacles** — ultrasonic detection and a detour guided by IMU and encoders
- **Ramps** — speed adjusted to the incline measured by the IMU

### Rescue
- **Victim detection** — YOLO model running on ONNX Runtime to find silver and black balls
- **Collection and delivery** — arm, wrist and gripper to store victims and drop them in the green and red zones
- **Rescue entry and exit** — silver-strip classifier (TFLite) to enter, visual zone recognition and odometry to leave

### Operation and safety
- **Live dashboard** — cameras, mission phases, commands and telemetry over HTTP/WebSocket
- **Fail-safe drive** — emergency stop has top priority, and the ESP32 stops the motors if commands stop arriving for 500 ms

## Hardware

- **Main computer**: Raspberry Pi
- **Microcontroller**: ESP32, connected over UART at 115200 bps
- **Vision**: down-facing and front-facing cameras
- **Drive**: four DC motors with DRV8833 drivers and wheel encoders
- **Sensors**: MPU6050 IMU and front ultrasonic sensor
- **Manipulator**: arm, wrist and gripper servos on a PCA9685
- **Interface**: SSD1306 OLED, start button and battery monitor

## Software Stack

- **Control**: C++17 and CMake
- **Vision**: Python 3, Picamera2, OpenCV and NumPy
- **Inference**: ONNX Runtime (YOLO) and LiteRT (TFLite)
- **Firmware**: ESP32 (Arduino)
- **Runtime**: systemd services on Linux, deployed over SSH

## Project Structure

```
soul-robot/
├── src/             # Application, missions, control, hardware layer, telemetry and dashboard
├── include/obr/     # C++ interfaces and configuration
├── esp32/           # ESP32 firmware (main and bench bridge)
├── scripts/         # Vision processes, run and deploy scripts
├── assets/          # Dashboard assets and inference models
├── deployment/      # systemd service installation
├── calibration/     # Line calibration data and reports
├── training/        # Silver-strip classifier training
├── tests/           # C++ and Python tests
└── docs/            # Technical documentation
```

## Getting Started

On the Raspberry Pi:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

Dependencies, wiring, calibration and deployment are covered in the [technical overview](docs/VISAO_TECNICA.md) (Portuguese) and the [ESP32 firmware guide](esp32/obr_esp32_main/README.md).

## Documentation

- [Technical overview](docs/VISAO_TECNICA.md) — architecture, safety, build and deploy
- [Hardware and operation guide](docs/GUIA_COMPLETO_DO_PROJETO.md)
- [ESP32 firmware and UART protocol](esp32/obr_esp32_main/README.md)
- [Rescue room routine](docs/ROTINA_SALA_RESGATE.md) and [exit sequence](docs/RESCUE_EXIT_SEQUENCE.md)
- [Obstacle avoidance](docs/OBSTACLE_AVOIDANCE.md)
- [Victim vision](scripts/ball_vision/README.md)

---

Built by [ORC Robotics](https://github.com/ORC-Robotics) at SENAI ORC, Brazil.
