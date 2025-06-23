# 🛠️ Board_V1 - STM32 Project

## 📋 Overview
This project is developed to test and validate the custom STM32 hardware board version 1. It includes initialization of basic peripherals such as GPIO, UART, SPI, and timers.

---

## 🧠 Project Details

- **MCU**: STM32F103C8T6 (Bluepill)
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