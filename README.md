# STM32 Motor Control Project (FreeRTOS)

This project implements a simple motor‑control system using **STM32 HAL**, **FreeRTOS**, and **PWM**.  
It reads accelerator pedal position and engine temperature using ADC, processes the data in RTOS tasks, and drives a motor using PWM output.

---

## 🚀 Features

### **Tasks**
- **PedalTask**  
  Reads ADC channel PA0 and converts pedal position to a percentage.

- **TemperatureTask**  
  Reads ADC channel PA1 and converts raw ADC values to °C.

- **ControlTask**  
  Combines pedal + temperature data and computes desired motor RPM.  
  Sends formatted telemetry over UART.

- **PWMTask**  
  Receives RPM commands and updates TIM3 CH1 PWM duty cycle.

---

## 📡 Peripherals Used
- **ADC1** – PA0 (pedal), PA1 (temperature)
- **UART2** – 115200 baud
- **TIM3 PWM** – CH1, 1 kHz PWM
- **GPIO** – Analog inputs

---

## 📦 RTOS Resources
- **Message Queues**
  - `PedalQueue`
  - `TemperatureQueue`
  - `MotorCommandQueue`

- **Mutex**
  - `ADC1Mutex` (protects ADC access)

---

## 🧮 Motor RPM Mapping
| Pedal (%) | RPM |
|-----------|-----|
| 0%        | 0   |
| 50%       | 1500 |
| 100%      | 3000 |

PWM duty cycle is mapped linearly to RPM:


---

## 🛠 Build Instructions
1. Open project in **STM32CubeIDE**  
2. Ensure FreeRTOS is enabled  
3. Build & flash to your STM32 board  
4. Open a serial terminal at **115200 baud** to view telemetry

---

