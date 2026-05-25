# A-Path-Planning-on-Real-Robot
BÁO CÁO TỔNG QUÁT CHO MÔN CƠ SỞ TRÍ TUỆ NHÂN TẠO, ĐƯỢC MỞ RỘNG RA TÍCH HỢP SOCIAL COSTMAP CHO MÔN CÁC VẤN ĐỀ HIỆN ĐẠI CỦA KỸ THUẬT MÁY TÍNH.
THUẬT TOÁN A* THÔNG MINH CHO ROBOT KẾT HỢP SOCIAL COSTMAP LÀM ROBOT DI CHUYỂN TẠO TÂM LÝ AN TÂM CHO CON NGƯỜI

<img width="645" height="812" alt="image" src="https://github.com/user-attachments/assets/30da6acc-2cce-415d-81c2-e3791f8b12fb" />

<img width="661" height="957" alt="image" src="https://github.com/user-attachments/assets/0663db85-6748-4a7a-abb4-f874433a6da5" />

# STM32 Line Following Autonomous Rover 🚗

## 📌 Project Overview

A full-stack embedded systems project featuring a high-speed autonomous line-following rover built around the STM32F401RE microcontroller. The system integrates analog IR sensors, dual DC motors, PWM motor control, and UART-based wireless telemetry for real-time monitoring and control.

The rover continuously reads sensor data, calculates line position error using a weighted algorithm, and applies a PD controller to maintain stable movement along a predefined track. The system also supports automatic sensor calibration, lost-line recovery, and finish-line detection.

---

## 🚀 Key Features

- **High-Speed Line Following**
  - Real-time line tracking using an 8-channel analog IR sensor array.

- **PD Control Algorithm**
  - Smooth and stable movement using proportional–derivative control.

- **Automatic Sensor Calibration**
  - Dynamic threshold calibration for improved accuracy under different lighting conditions.

- **Lost-Line Recovery**
  - Intelligent recovery algorithm using previous line direction memory.

- **Finish-Line Detection**
  - Detects destination area when all sensors simultaneously detect the line.

- **UART Telemetry**
  - Real-time transmission of sensor values and control error for debugging and monitoring.

- **STM32 Bare-Metal Programming**
  - Direct register-level programming without HAL libraries.

---

## 🛠 Hardware Components

- **MCU:** STM32F401RE Nucleo-64
- **Motor Driver:** BTS7960 High-Power H-Bridge
- **Motors:** GA25-300 DC Gear Motors (1300 RPM)
- **Sensors:** 8-Channel Analog IR Line Sensor
- **Wireless Communication:** ESP8266 Wi-Fi Module
- **Power Supply:**
  - 18650 Li-ion Battery Pack
  - XL4015 Buck Converter

---

## 🧠 System Architecture

The rover system is divided into five major functional blocks:

1. **Power Supply Block**
   - Supplies stable voltage to motors and control logic.

2. **Processing Block**
   - STM32F401RE handles sensor acquisition and control algorithms.

3. **Sensor Block**
   - Analog IR sensors detect black line position.

4. **Motor Control Block**
   - BTS7960 drivers receive PWM signals to control motor speed and direction.

5. **Communication Block**
   - ESP8266 transmits telemetry data wirelessly.

```text
[-3500, -2500, -1500, -500,
  500,  1500,  2500, 3500]
