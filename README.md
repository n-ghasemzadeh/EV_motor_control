# EV Motor Control – Completed Version (EV_motor_control_CAN1)

This folder contains the **completed and production‑ready version** of the EV Motor Control project.  
It expands the original design with a full FreeRTOS architecture, CAN command handling, ADC‑based pedal and temperature sensing, PWM motor control, and UART diagnostics.

This version lives in a different directory and project name and is added to the repository as a **separate branch** to preserve workflow history.

---

## 🚗 Project Overview

This project implements a complete STM32‑based EV motor control system using:

- **STM32 Nucleo‑F446RE**
- **CAN1 peripheral** with SN65HVD230 transceiver
- **FreeRTOS** for deterministic task scheduling
- **ADC1** for pedal and temperature sensing
- **TIM3 PWM** for motor speed control
- **UART2** for runtime diagnostics
- **Safety logic** for CAN timeout, invalid commands, and over‑temperature

The system receives commands over CAN (ID **0x200**) and publishes status frames (ID **0x100**) at a fixed period.

---

## 🧩 System Architecture

### FreeRTOS Tasks

| Task Name          | Purpose |
|--------------------|-------------------------------------------------------------|
| **PedalTask**      | Reads ADC Channel 0 → pedal percentage (0–100%). |
| **TemperatureTask**| Reads ADC Channel 1 → temperature (scaled to °C). |
| **ControlTask**    | Core logic: safety checks, RPM calculation, UART debug. |
| **PWMTask**        | Converts desired RPM → TIM3 PWM duty cycle. |
| **CanTask**        | Receives CAN commands, validates frames, updates state. |
| **DefaultTask**    | Idle task. |

---


## ⏱ FreeRTOS Task Timing Diagram

```mermaid id="r4z1kv"
gantt
    title EV Motor Control - FreeRTOS Task Timing
    dateFormat X
    axisFormat %L

    section PedalTask (10 ms)
    ADC Read 1 :active, 0, 10
    ADC Read 2 :active, 10, 20
    ADC Read 3 :active, 20, 30
    ADC Read 4 :active, 30, 40
    ADC Read 5 :active, 40, 50
    ADC Read 6 :active, 50, 60
    ADC Read 7 :active, 60, 70
    ADC Read 8 :active, 70, 80
    ADC Read 9 :active, 80, 90
    ADC Read 10 :active, 90, 100

    section TemperatureTask (100 ms)
    Temperature ADC Read :active, 0, 100

    section ControlTask (20 ms)
    Control Cycle 1 :active, 0, 20
    Control Cycle 2 :active, 20, 40
    Control Cycle 3 :active, 40, 60
    Control Cycle 4 :active, 60, 80
    Control Cycle 5 :active, 80, 100

    section CanTask
    CAN RX Polling :crit, 0, 100
    Status TX :active, 90, 100

    section PWMTask
    Apply PWM :active, 5, 10
```


## 📡 CAN Protocol

### Incoming Command Frame (StdID = 0x200, DLC = 8)

| Byte | Meaning                          |
|------|----------------------------------|
| 0    | Enable (0 or 1)                  |
| 1–2  | Max RPM limit (uint16)           |
| 3    | Mode (must be 0)                 |
| 4–7  | Reserved (must be 0)             |

Invalid frames trigger:

- `FAULT_INVALID_COMMAND`
- `canEnableRequest = 0`
- `canMaxRPM = 0`

### Outgoing Status Frame (StdID = 0x100, DLC = 8)

| Byte | Meaning                                  |
|------|------------------------------------------|
| 0–1  | Requested RPM                            |
| 2–3  | Measured RPM (0xFFFF placeholder)        |
| 4–5  | Temperature × 0.1°C                      |
| 6    | Status flags                             |
| 7    | Fault code                               |



## 📡 CAN Protocol Diagram

```mermaid
flowchart TD
    A["CAN Bus"] -->|"StdID 0x200, DLC = 8"| B["STM32 CAN RX (CanTask)"]

    subgraph Incoming["Incoming Command Frame (0x200)"]
        C0["Byte 0: Enable (0 or 1)"]
        C1["Bytes 1-2: Max RPM Limit (uint16)"]
        C2["Byte 3: Mode (must be 0)"]
        C3["Bytes 4-7: Reserved (must be 0)"]
    end

    B --> C0
    B --> C1
    B --> C2
    B --> C3

    C0 --> D["Validate Command"]
    C1 --> D
    C2 --> D
    C3 --> D

    D -->|Valid| E["Update State: canEnableRequest, canMaxRPM, lastCanCommandTick"]
    D -->|Invalid| F["Set Fault: FAULT_INVALID_COMMAND"]

    E --> G["ControlTask: RPM Calculation"]
    F --> G

    G --> H["PWMTask: Apply PWM Duty Cycle"]

    H -->|"Periodic 100 ms"| I["STM32 CAN TX (CAN_TransmitStatus)"]

    subgraph Outgoing["Outgoing Status Frame (0x100)"]
        S0["Bytes 0-1: Requested RPM"]
        S1["Bytes 2-3: Measured RPM"]
        S2["Bytes 4-5: Temperature x 0.1 C"]
        S3["Byte 6: Status Flags"]
        S4["Byte 7: Fault Code"]
    end

    I --> S0
    I --> S1
    I --> S2
    I --> S3
    I --> S4

    I -->|"StdID 0x100, DLC = 8"| A

```


## ⚙️ Motor Control Logic Flowchart

```mermaid
flowchart TD
    A["PedalTask: pedalPercent"] --> C
    B["TemperatureTask: temperatureC"] --> C
    X["CanTask: enable, maxRPM, lastCommandTick, faultCode"] --> C

    C["ControlTask: 20 ms loop"] --> D{"Over-temperature?"}

    D -->|Yes| F["Set OVERTEMPERATURE fault<br/>Disable motor"]
    D -->|No| E{"CAN timeout or no command?"}

    E -->|Yes| G["Set CAN_TIMEOUT fault<br/>Disable motor"]
    E -->|No| H{"Command valid?"}

    H -->|No| I["Set INVALID_COMMAND fault<br/>Disable motor"]
    H -->|Yes| J["Compute pedalRPM = pedalPercent × MAX_RPM"]

    J --> K["Clamp pedalRPM to CAN maxRPM"]
    K --> L{"Motor enabled?"}

    L -->|Yes| M["finalRPM = pedalRPM"]
    L -->|No| N["finalRPM = 0"]

    M --> O["Update requestedRPMStatus"]
    N --> O

    O --> P["Reset MotorCommandQueue"]
    P --> R["Send desiredSpeedRPM to PWMTask"]

    R --> S["PWMTask: Apply PWM duty cycle"]
    S --> T{"timingPending?"}

    T -->|Yes| U["Compute CAN-to-PWM latency<br/>UART print"]
    T -->|No| V["Continue"]

    S --> W["Transmit CAN status every 100 ms"]
```




## 🧪 Testing Workflow

1. Power the Nucleo‑F446RE and the SN65HVD230 CAN transceiver.
2. Ensure the CAN bus is properly terminated with a 120Ω resistor.
3. Connect the **Jhoinrch CAN interface** between the STM32 board and your PC.
4. Use **Cangaroo** software on the PC to send and monitor CAN frames:
   - Transmit command frames (StdID = 0x200, DLC = 8)
   - Observe status frames (StdID = 0x100, DLC = 8)
5. Open a serial terminal at **115200 baud** to view UART diagnostics from USART2.
6. Verify PWM output on **TIM3 Channel 1** using a **logic analyzer**:
   - Confirm duty cycle changes with pedal input
   - Confirm RPM clamping based on CAN maxRPM
7. Monitor pedal ADC readings (Channel 0) and temperature ADC readings (Channel 1) to confirm correct scaling.
8. Validate safety features:
   - CAN timeout (500 ms)
   - Over‑temperature shutdown (≥ 90°C)
   - Invalid command rejection
9. Confirm periodic CAN status frames (StdID = 0x100, DLC = 8) are transmitted every 100 ms.

## 📈 Performance & Test Results

- Successfully tested **30,000+ CAN frames** using the Jhoinrch CAN interface and Cangaroo software with **zero observed errors**.
- Measured **CAN‑command‑to‑control‑task response latency** between **4–14 ms**, depending on system load and task scheduling.
- Verified PWM output timing and duty‑cycle accuracy using a **logic analyzer** connected to TIM3 Channel 1.
- Confirmed stable FreeRTOS task execution:
  - PedalTask at 10 ms
  - ControlTask at 20 ms
  - TemperatureTask at 100 ms
  - CAN status transmission every 100 ms
- No missed dealines or queue overflows observed during extended testing.



## 📁 Repository Structure

```text
EV_motor_control/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── Middlewares/
├── .settings/
├── .gitignore
├── .project
├── .cproject
├── EV_motor_control_CAN1l.ioc
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
└── README.md
```

- **`completed-version` branch:** Completed EV motor-control project with CAN communication, FreeRTOS tasks, and PWM control.
- **`main` branch:** Original project version.
```







