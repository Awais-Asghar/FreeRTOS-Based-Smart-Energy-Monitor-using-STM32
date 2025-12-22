# FreeRTOS-Based Smart Energy Monitor using STM32
![Status](https://img.shields.io/badge/Status-Completed-success)
![Platform](https://img.shields.io/badge/Platform-Embedded-blue)
![MCU](https://img.shields.io/badge/MCU-STM32F746%20%7C%20ESP32-pink)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange)
![Cloud](https://img.shields.io/badge/Cloud-Firebase-yellow)
![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-red)
![Lisence](https://img.shields.io/badge/Lisence-MIT-lightgrey)

## Project Overview

This project implements a **real-time smart energy monitoring and billing system** using the **STM32F746 microcontroller** and **ESP32**. The system measures voltage and current, computes RMS values, calculates power and energy consumption, and uploads live data to the **Firebase cloud** using WiFi. The design follows a **FreeRTOS-based multi-task architecture**, ensuring deterministic operation and clean separation of responsibilities.

---

## Key Features

- Real-time voltage and current measurement  
- RMS calculation with DC offset removal  
- Power and energy (kWh) computation  
- Cloud-based monitoring using Firebase  
- FreeRTOS-based multi-threaded design  
- UART communication between STM32 and ESP32  

---

## System Architecture

- **Voltage Sensor (ZMPT101B)** → STM32 ADC  
- **Current Sensor (ACS712 / INA219)** → STM32 ADC  
- **STM32F746** → Signal processing + RMS calculation (FreeRTOS)  
- **ESP32** → WiFi + Firebase cloud upload  
- **UART** → Communication between STM32 and ESP32  

---

## Tools and Technologies

- STM32F746  
- FreeRTOS  
- STM32CubeIDE  
- IAR Embedded Workbench  
- ESP32  
- Arduino IDE  
- Firebase Realtime Database  
- UART Communication  
- C / C++  

---

## Core Calculations

- **Current RMS**
\[
I_{rms} = \sqrt{\frac{1}{N} \sum (i_n - \bar{i})^2}
\]

- **Power**
\[
P = V_{rms} \times I_{rms} \times PF
\]

- **Energy**
\[
Energy (kWh) = \frac{P \times t}{1000}
\]

- **Billing**
\[
Bill = Energy \times CostPerUnit
\]

---

## FreeRTOS Task Design

### STM32 (RTOS-based)
- ADC Sampling Task (High Priority)  
- RMS & Signal Processing  
- UART Transmission Task  

### ESP32 (Dual-Core FreeRTOS)
- UART Receive Task  
- Cloud Upload Task  
- Serial Debug Task  

Mutexes are used to protect shared data across tasks.

---

## Results

- Stable RMS voltage and current measurement  
- Accurate power and energy computation  
- Live cloud updates every 5 seconds  
- Robust real-time performance using FreeRTOS  

---

## Applications

- Smart home energy monitoring  
- Industrial power usage tracking  
- Smart metering systems  
- Energy consumption analysis  

---

## Team Members

- Awais Asghar  
- Muhammad Haris  
- Saad Farooq  
- Aneeqa Ur Rehman  

---

## References

- STM32F746 Reference Manual  
- ESP32 Technical Reference  
- Firebase Realtime Database Documentation  
- FreeRTOS Official Documentation  

---

## License

This project is intended for **academic and educational use**.
