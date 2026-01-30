# BNO055 IMU Acquisition

This repository contains an Arduino sketch for orientation data acquisition using the **BNO055 inertial measurement unit (IMU)** and an **Arduino Nano ESP32**.

The system is designed as a simple and reliable platform to obtain **roll, pitch, and yaw** orientation angles, intended for preliminary validation in **biomedical motion monitoring applications**.

---

## Overview

The sketch initializes and configures the BNO055 sensor, acquires absolute orientation data, and outputs the estimated angles in real time.  
Basic calibration handling and signal stabilization strategies are implemented to improve measurement consistency, especially during static and slow movements.

---

## Hardware
- Arduino Nano ESP32  
- BNO055 Absolute Orientation Sensor  

---

## Software
- Arduino IDE  
- Required libraries (installed via Arduino Library Manager):
  - Adafruit BNO055  
  - Adafruit Unified Sensor  
  - ArduinoBLE  

---

## How to run
1. Open `IMUs_Acquisition.ino` in the Arduino IDE  
2. Select **Arduino Nano ESP32** as the target board  
3. Connect the BNO055 via **I2C** (SDA, SCL, VCC, GND)  
4. Upload the sketch to the board  
5. Open the Serial Monitor at the configured baud rate to visualize orientation data  

---

## Functionality
- Acquisition of absolute orientation data from the BNO055  
- Computation of roll, pitch, and yaw angles  
- Sensor calibration handling  
- Signal stabilization through filtering strategies  
- Serial (and/or BLE) data output for visualization and analysis  

---

## Repository contents
- `IMUs_Acquisition.ino` – Main Arduino sketch implementing the IMU acquisition system  

---

## Notes
This project is intended for academic and experimental use.  
The code is provided as a standalone sketch and relies on external libraries maintained by their respective authors.
