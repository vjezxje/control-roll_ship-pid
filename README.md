This is my graduation project, developed as part of my final-year thesis.
The project involves designing and implementing a custom embedded system using the STM32F407VGT6 microcontroller.
It focuses on initializing and managing various peripherals such as GPIO, UART, and SPI, and demonstrates the core functions of a custom hardware board (Board_V1).
The project was developed using STM32CubeIDE, STM32CubeMX and STM32CubeMonitor.

## 📋 Overview
This project is developed to test and validate the custom STM32 hardware board version 1. It includes initialization of basic peripherals such as GPIO, UART, SPI, and timers.

---

## 🧠 Project Details

- **MCU**: TM32F407VGT6 (Bluepill)
- **Clock Speed**: 72MHz
- **IDE**: STM32CubeIDE
- **Toolchain**: GNU ARM (gcc)
- **Peripherals Used**:
  - UART (for serial debug)
  - GPIO (for LED and button)
  - SPI (optional sensor/expansion)

---

## 📂 Folder Structure

Board_V1/
├── Core/

├── Drivers/

├── Inc/ <-- Optional

├── Src/

├── Board_V1.ioc <-- STM32CubeMX config file

├── .gitignore

└── README.md
