# IMU Orientation Acquisition – BNO055

This repository contains an Arduino sketch for orientation data acquisition using the **BNO055 inertial measurement unit (IMU)** and an **Arduino Nano ESP32**.

The system is designed as a simple and reliable platform to obtain **roll, pitch, and yaw** orientation angles, intended for preliminary validation in **biomedical motion monitoring applications**.

---

## Overview

The sketch initializes and configures the BNO055 sensor, acquires orientation data, and outputs the estimated angles in real time.  
Basic calibration handling and signal stabilization strategies are implemented to improve measurement consistency during static and slow movements.

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

## Functionality
- Acquisition of absolute orientation data from the BNO055  
- Computation of roll, pitch, and yaw angles  
- Sensor calibration handling  
- Signal stabilization through filtering strategies  
- Serial (and/or BLE) data output for visualization and analysis  

---

## Repository Contents
- `IMUs_Acquisition.ino` – Main Arduino sketch implementing the IMU acquisition system

---

## Notes
This project is intended for academic and experimental use.  
The code is provided as a standalone sketch and relies on external libraries maintained by their respective authors.

