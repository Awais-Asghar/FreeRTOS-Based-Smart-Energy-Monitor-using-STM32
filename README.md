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

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/68d03d54-7ac5-4df1-9eaf-bc7bd773f192" />
<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/568e7c28-0849-44a2-9208-fbdad3808a4c" />
<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/3277db49-f717-4398-97e5-645a5e0383a9" />
<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/8210f5a9-0ab0-4f7a-850a-6395c28fd794" />

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

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/492c7a86-4f1d-4fd9-b169-7114faec7d40" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/df08c564-7eb0-4118-9e1c-0cfbfa30ba79" />

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

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/39677225-12bf-4093-91ab-c30c6f2f734f" />

---

## Core Implementation 

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/02ad9740-2362-43a5-97c4-a579fc8e8cf5" />

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

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/a7c97ada-6c27-4410-9770-7f544bd89809" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/fcfff1ea-aa3b-4faa-8c73-9d5cd25a0897" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/9c0c27b8-1651-4e5e-929b-40c18122d055" />

---

## Hardware Implementation

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/6de8c5e1-3d40-42e4-b017-960eae6965fe" />

---

## Results

- Stable RMS voltage and current measurement  
- Accurate power and energy computation  
- Live cloud updates every 5 seconds  
- Robust real-time performance using FreeRTOS  

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/5e65149a-96e4-4a2f-b78f-0195a3317b8d" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/69ea2ebf-acbc-4d69-9620-0406c8ec76c8" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/c25b0867-d8c6-4965-be92-5310b40e9ae8" />

---

## Applications

- Smart home energy monitoring  
- Industrial power usage tracking  
- Smart metering systems  
- Energy consumption analysis  

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/aac3681e-aee5-4a2a-a970-ad773787d571" />

---

## Demo Video 

https://github.com/user-attachments/assets/38ebc3f2-3e4e-472e-ad77-98723180af24

## Team Members

- Awais Asghar  
- Muhammad Haris  
- Saad Farooq  
- Aneeqa Ur Rehman  

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/7befec41-7d42-432c-a2e2-155f0b0d006c" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/346af123-a0d1-416b-ba71-0dd4cace59d6" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/51f636ab-92c5-4b07-a5ca-efefb821ccfe" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/34b53777-6a60-48ee-9b24-819a07e18fd8" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/1cb026ce-8677-43c1-852c-a42b492bea64" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/4422d911-1edf-47b8-82a3-0826c85d0d0a" />

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/d75f2d7b-2a0e-4231-87b3-fd84247a93e1" />

---

## References

- STM32F746 Reference Manual  
- ESP32 Technical Reference  
- Firebase Realtime Database Documentation  
- FreeRTOS Official Documentation  

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/efe77b9d-cdf6-45f9-a92c-438f94f5b3c0" />

---

## License

This project is intended for **academic and educational use**.
