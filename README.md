# FreeRTOS-Based Flight Controller for FRDM-MCXn947

## Project Overview

This project implements a **flight controller** for the FRDM-MCXn947 development board using NXP's MCXn947 ARM Cortex-M33 microcontroller. The system uses **FreeRTOS** for real-time task scheduling and provides a complete architecture for autonomous drone flight control, with concurrent handling of radio control input, motor commands, and telemetry feedback.

**Target Hardware:**
- **MCU:** NXP MCXn947 (Cortex-M33, dual-core capable)
- **Development Board:** FRDM-MCXn947
- **Supported Motors:** 4× DShot-capable ESCs with telemetry feedback

---

## Architecture Overview

### System Design

The flight controller is built on a **task-based architecture** using FreeRTOS, with three primary concurrent tasks coordinated by timer-driven interrupts:

```
Timer ISR (800 Hz)
    ├─> DSHOTGeneratorTask (Motor Control)
    └─> signals RC & ESC tasks via DMA completion
         ├─> RCParserTask (Radio Control Input)
         ├─> ESCTelemetryTask (Motor Feedback)
         └─> [Future] SensorFusionTask
              └─> [Future] ControlAlgorithmTask
```

### Task Execution Timeline & Synchronization

```mermaid
sequenceDiagram
    actor TMR as LPTMR<br/>(800 Hz)
    participant DSHOT as DSHOTGeneratorTask
    participant RC as RCParserTask
    participant ESC as ESCTelemetryTask
    participant HW as Hardware<br/>Motors + Receiver

    TMR->>DSHOT: ISR: Task Notification
    activate DSHOT
    DSHOT->>DSHOT: Select motor_id (0-3)
    DSHOT->>DSHOT: Queue motor_id → escTelemetryMotorIdQueue
    DSHOT->>HW: DMA PWM: DShot frame + telemetry request
    deactivate DSHOT

    HW->>RC: RC frame ready (DMA)
    activate RC
    RC->>RC: Parse frame, validate CRC
    RC->>RC: Extract 14 channels
    deactivate RC

    HW->>ESC: ESC telemetry ready (DMA)
    activate ESC
    ESC->>ESC: Dequeue motor_id
    ESC->>ESC: Parse telemetry (10 bytes)
    ESC->>ESC: Store in esc→motorX[motor_id]
    deactivate ESC

    Note over DSHOT,ESC: Next cycle: Δt = 1.25 ms @ 800 Hz
```

### Task Interaction & Data Flow

```mermaid
graph TD
    A["🕐 LPTMR ISR<br/>(800 Hz)"] -->|vTaskNotifyGiveFromISR| B["⚙️ DSHOTGeneratorTask<br/>(Motor Control)"]
    
    B -->|Queue motor_id| C["📤 escTelemetryMotorIdQueue<br/>(FIFO, 8 slots)"]
    B -->|PWM + DMA| D["🔌 Motor 0-3 + UART Telemetry"]
    
    D -->|DMA RX Complete| E["📨 ESCTelemetryTask<br/>(Telemetry Decoder)"]
    E -->|Dequeue motor_id| C
    E -->|CRC8 Validate| F["💾 esc→motorX<br/>(dshotTelemetry_t)"]
    F -->|Temperature, Voltage,<br/>Current, RPM| G["📊 Motor Telemetry<br/>State Storage"]
    
    D -->|DMA RX Complete| H["📡 RCParserTask<br/>(RC Receiver)"]
    H -->|CRC Validate| I["🎮 RC 14-Channels<br/>Parsed"]
    I -->|CH1-CH4: RPYT<br/>CH5+: Mode| J["📥 Future: Control Loop"]
    
    J -->|PID + Sensor Fusion| K["🚁 Control Algorithm<br/>(Planned)"]
```

### Current Implementation: 3 Primary Tasks

#### 1. **RCParserTask** - Radio Control Receiver
- **Purpose:** Parses incoming RC frames from an FS-iA6B receiver
- **Protocol:** UART-based binary protocol (32-byte frames)
- **Key Features:**
  - Non-blocking reception via DMA (LPUART + eDMA)
  - CRC validation for frame integrity
  - ISR-based notification system (DMA completion triggers task)
  - Handles all 14 RC channels from the receiver
  - Automatically resynchronizes on corrupted frames
- **Data:** `fs_ia6b_channels_t` containing 14-channel (CH1-CH14) PWM values (800-2200 µs)
- **Status:** ✅ Fully implemented and tested

#### 2. **DSHOTGeneratorTask** - Motor Control
- **Purpose:** Sends DShot protocol commands to 4 electronic speed controllers (ESCs)
- **Protocol:** DShot600 (600 kbps) via PWM
- **Key Features:**
  - Timed at 800 Hz (every 1.25 ms) via LPTMR interrupt
  - Cycles through motors 0→1→2→3, requesting telemetry from each
  - Throttle ramping with CRC8 packet validation
  - DMA-driven PWM signal generation (maintains MCU in low-power mode)
  - Requests telemetry from one motor per cycle for bandwidth efficiency
- **Motor Structure:** `dshotMotor_t` per motor (16 total fields for control + feedback)
- **Status:** ✅ Fully implemented with telemetry queue integration

#### 3. **ESCTelemetryTask** - Motor Telemetry Decoder
- **Purpose:** Receives and parses telemetry data from ESCs
- **Protocol:** DShot telemetry (10-byte frames at 115.2 kbaud UART)
- **Key Features:**
  - Non-blocking DMA-based UART reception
  - CRC8 validation for frame integrity
  - **Queue-based motor ID tracking:** Knows which motor sent each telemetry frame via `escTelemetryMotorIdQueue`
  - Stores decoded telemetry directly into corresponding `esc->motorX.dshot_telemtry` struct
  - Extracts: temperature (°C), voltage (centivolts), current (centiamps), power consumption (mAh), eRPM
- **Status:** ✅ Fully implemented (queue integration added)

---

## Hardware & Peripheral Mapping

### Communication Interfaces (DMA-Enabled)

| Peripheral | Purpose | Protocol | DMA | Status |
|------------|---------|----------|-----|--------|
| **LPUART** | RC receiver input | Binary protocol @ ~9.6 kbaud | ✅ eDMA | ✅ Active |
| **LPUART** | ESC telemetry | DShot @ 115.2 kbaud | ✅ eDMA | ✅ Active |
| **LPSPI** | IMU sensor (accel/gyro) | SPI/DMA | ❌ TODO | 🚧 Planned |
| **LPSPI** | Barometer (pressure/temp) | SPI/DMA | ❌ TODO | 🚧 Planned |

### Control Interfaces

| Peripheral | Purpose | Frequency | Status |
|------------|---------|-----------|--------|
| **PWM** (eTPU) | DShot motor control (4×) | 600 kHz carrier | ✅ Active |
| **eDMA** | PWM pattern generation | N/A (driven by timer) | ✅ Active |
| **LPTMR** | Task scheduler timer | 800 Hz | ✅ Active |

---

## Data Flow & Queue Architecture

### Motor Telemetry Pairing System

The **escTelemetryMotorIdQueue** ensures telemetry frames are stored in the correct motor struct:

```mermaid
sequenceDiagram
    participant DSHOT as DSHOTGeneratorTask<br/>@ 800 Hz
    participant Q as escTelemetryMotorIdQueue<br/>(FIFO: motor_id)
    participant PWM as DShot Motors<br/>(0-3)
    participant ESC as ESCTelemetryTask
    participant Storage as esc→motorX<br/>.dshot_telemtry

    rect rgb(200, 220, 255)
    Note over DSHOT,Storage: Cycle 1: Motor 0
    DSHOT->>Q: xQueueSendToBack(Q, &motor_id=0)
    Q->>Q: Queue[0] ← 0
    DSHOT->>PWM: DMA: DShot(throttle, telemetry_request=true)
    end
    
    rect rgb(200, 220, 255)
    Note over DSHOT,Storage: Cycle 2: Motor 1 (5-10ms later)
    PWM->>ESC: UART DMA RX: Motor 0 telemetry (10 bytes)
    ESC->>Q: xQueueReceive(Q, &motor_id) → motor_id=0
    Q->>Q: Dequeue, Queue[0] removed
    ESC->>Storage: dshot_process_telemetry(&esc→motor0, buffer)
    ESC->>Storage: esc→motor0.dshot_telemtry.temperature = buffer[0]
    
    DSHOT->>Q: xQueueSendToBack(Q, &motor_id=1)
    DSHOT->>PWM: DMA: DShot(throttle, motor_id=1, telemetry_request=true)
    end
```

### Queue-based Motor Identification Timing

```mermaid
timeline
    title 800 Hz Motor Control Cycle (4 motors, each gets 1.25 ms)

    section Motor Sequencing
    t=0ms : Cycle 0 : Motor 0 : m0_id queued
    t=1.25ms : Cycle 1 : Motor 1 : m1_id queued
    t=2.5ms : Cycle 2 : Motor 2 : m2_id queued
    t=3.75ms : Cycle 3 : Motor 3 : m3_id queued
    t=5ms : Cycle 0 (repeat) : Motor 0 : m0_id queued again

    section Telemetry Response (τ ≈ 5-10ms)
    t=5-10ms : Motor 0 Response : ESC telemtry RX : m0_id dequeued
    t=6-12ms : Motor 1 Response : ESC telemetry RX : m1_id dequeued
    t=7-13ms : Motor 2 Response : ESC telemetry RX : m2_id dequeued
    t=8-15ms : Motor 3 Response : ESC telemetry RX : m3_id dequeued
```

### Task Synchronization

- **Task notifications** via `vTaskNotifyGiveFromISR()` for low-latency wakeups
- **Queues** for passing data between tasks (RC channels, motor IDs)
- **Global synchronization** through shared `dshotSystem_t esc` pointer

---

## System Control Flow & Initialization

```mermaid
stateDiagram-v2
    [*] --> BOARD_Init: Power-up
    BOARD_Init --> NVIC_Config: Configure ISR Priorities<br/>(eDMA, LPTMR)
    NVIC_Config --> HW_INIT: Initialize Peripherals
    
    HW_INIT --> RC_INIT: rc_init()<br/>LPUART + DMA setup
    RC_INIT --> DSHOT_INIT: dshot_init(4 motors)<br/>PWM + telemetry UART
    
    DSHOT_INIT --> TIMER_INIT: timer_init(800 Hz)<br/>LPTMR config
    TIMER_INIT --> QUEUE_CREATE: Create FreeRTOS Queues<br/>rc, esc, motor_id
    
    QUEUE_CREATE --> TASK_CREATE: xTaskCreate() × 3<br/>RC, ESC, DSHOT tasks
    TASK_CREATE --> SCHED_START: vTaskStartScheduler()
    
    SCHED_START --> RUNNING: Tasks + IRQs<br/>operating
    
    RUNNING --> RC_LOOP: RCParserTask<br/>event-driven
    RC_LOOP --> RC_LOOP: Wait for RC DMA<br/>→ parse frame<br/>→ validate CRC<br/>→ restart DMA
    
    RUNNING --> ESC_LOOP: ESCTelemetryTask<br/>event-driven
    ESC_LOOP --> ESC_LOOP: Wait for ESC DMA<br/>→ dequeue motor_id<br/>→ parse & validate<br/>→ store telemetry
    
    RUNNING --> DSHOT_LOOP: DSHOTGeneratorTask<br/>timer-driven (800 Hz)
    DSHOT_LOOP --> DSHOT_LOOP: Wait for timer ISR<br/>→ queue motor_id<br/>→ send DShot frame<br/>→ request telemetry
    
    RUNNING --> LPTMR_ISR: LPTMR0 ISR @ 800 Hz
    LPTMR_ISR --> DSHOT_LOOP: vTaskNotifyGiveFromISR()<br/>→ wake DSHOTGeneratorTask
```

---

## Implemented Features

### ✅ Complete

- [x] RC receiver integration (FS-iA6B, 14-channel IBUS protocol)
- [x] DShot600 motor control with PWM + DMA
- [x] ESC telemetry decoding (10-byte frame, CRC8 validation)
- [x] Motor-specific telemetry storage (queue-based ID tracking)
- [x] Non-blocking DMA-based UART I/O (RC & ESC)
- [x] Timer-driven task scheduling (800 Hz)
- [x] FreeRTOS task management with proper priorities
- [x] Interrupt priority configuration for nested critical sections

### 🚧 In Progress / TODO

#### Phase 2: Sensor Integration
- [ ] **IMU (Inertial Measurement Unit)**
  - LPSPI + DMA for accelerometer & gyroscope & magnetometer
  - 9-DOF measurements (250Hz)
  - Support for common sensors: ICM-20948, GY-BNO085

- [ ] **Barometer**
  - LPSPI + DMA for pressure & temperature sensing
  - Altitude estimation via pressure-to-altitude conversion
  - BME280

#### Phase 3: Sensor Fusion
- [ ] **SensorFusionTask** (new task)
  - Fuse IMU and barometer data
  - Complementary filter or Kalman filter implementation
  - Output: attitude (roll, pitch, yaw), altitude, vertical velocity

#### Phase 4: Control Algorithm
- [ ] **ControlLoopTask ** (new task)
  - **PID control loops:**
    - Attitude stabilization (roll, pitch, yaw rates)
    - Altitude hold 
  - Read RC inputs → fuse sensor data → compute motor commands
  - 400 Hz loop
  - **Failsafe:** descending throttle ramp on signal loss

---

## Data Structures & Protocol Flow

### Data Structure Hierarchy

```mermaid
graph TB
    ROOT["🚁 dshotSystem_t<br/>(Global ESC System)"]
    
    ROOT --> UART["uart_ctrl_t<br/>(Telemetry UART)"]
    ROOT --> M0["dshotMotor_t<br/>Motor 0"]
    ROOT --> M1["dshotMotor_t<br/>Motor 1"]
    ROOT --> M2["dshotMotor_t<br/>Motor 2"]
    ROOT --> M3["dshotMotor_t<br/>Motor 3"]
    
    M0 --> TELEM0["dshotTelemetry_t<br/>(Feedback)"]
    M0 --> CTRL0["dshotControl_t<br/>(Commands)"]
    M0 --> DMA0["dma_ctrl_t<br/>(PWM buffer)"]
    M0 --> PWM0["pwm_ctrl_t<br/>(PWM config)"]
    M0 --> ID0["motor_id: 0"]
    
    M1 --> TELEM1["dshotTelemetry_t"]
    M1 --> CTRL1["dshotControl_t"]
    M1 --> DMA1["dma_ctrl_t"]
    M1 --> PWM1["pwm_ctrl_t"]
    M1 --> ID1["motor_id: 1"]
    
    M2 --> TELEM2["dshotTelemetry_t"]
    M2 --> CTRL2["dshotControl_t"]
    M2 --> DMA2["dma_ctrl_t"]
    M2 --> PWM2["pwm_ctrl_t"]
    M2 --> ID2["motor_id: 2"]
    
    M3 --> TELEM3["dshotTelemetry_t"]
    M3 --> CTRL3["dshotControl_t"]
    M3 --> DMA3["dma_ctrl_t"]
    M3 --> PWM3["pwm_ctrl_t"]
    M3 --> ID3["motor_id: 3"]
    
    TELEM0 --> TEMP["temperature_u8"]
    TELEM0 --> VOLT["voltage_cv_u16"]
    TELEM0 --> AMP["current_ca_u16"]
    TELEM0 --> MAH["consumption_mah_u16"]
    TELEM0 --> RPM["erpm_u16"]
    TELEM0 --> VAL["valid_b"]
    
    CTRL0 --> THROT["throttle_u16"]
    CTRL0 --> TREQ["requestTelemetry_b"]
    
    style ROOT fill:#ff9999
    style M0 fill:#99ccff
    style M1 fill:#99ccff
    style M2 fill:#99ccff
    style M3 fill:#99ccff
    style TELEM0 fill:#99ff99
    style CTRL0 fill:#ffff99
```

### Protocol Communication Layers

```mermaid
graph LR
    subgraph RC["🎮 RC Receiver Path"]
        RC_HW["Receiver<br/>FS-iA6B"]
        RC_UART["LPUART + DMA<br/>9.6 kbaud"]
        RC_BUF["g_rc_rxBuffer<br/>32 bytes"]
        RC_PARSE["rc_parse_frame()"]
        RC_DATA["fs_ia6b_channels_t<br/>14 channels"]
    end
    
    subgraph MOTOR["⚙️ Motor Control Path"]
        THROTTLE["Throttle<br/>0-2047"]
        DSHOT_PKT["dshot_prepare_packet()"]
        DMA_BUF["dma_buf[18]<br/>PWM pattern"]
        DMA_TX["eDMA<br/>DShot600"]
        MOTOR_HW["Motor 0-3<br/>ESCs"]
    end
    
    subgraph ESC["📊 ESC Telemetry Path"]
        ESC_HW["ESC Response<br/>DShot Packet"]
        ESC_UART["LPUART + DMA<br/>115.2 kbaud"]
        ESC_BUF["g_esc_rxBuffer<br/>10 bytes"]
        CRC_CHK["get_crc8()<br/>validate"]
        TELEM_DATA["dshotTelemetry_t<br/>5 metrics"]
        MOTOR_STRU["esc→motorX<br/>.dshot_telemtry"]
    end
    
    RC_HW --> RC_UART
    RC_UART --> RC_BUF
    RC_BUF --> RC_PARSE
    RC_PARSE --> RC_DATA
    
    THROTTLE --> DSHOT_PKT
    DSHOT_PKT --> DMA_BUF
    DMA_BUF --> DMA_TX
    DMA_TX --> MOTOR_HW
    
    MOTOR_HW --> ESC_HW
    ESC_HW --> ESC_UART
    ESC_UART --> ESC_BUF
    ESC_BUF --> CRC_CHK
    CRC_CHK --> TELEM_DATA
    TELEM_DATA --> MOTOR_STRU
    
    style RC fill:#e1f5ff
    style MOTOR fill:#fff3e0
    style ESC fill:#f3e5f5
```

## Data Structures

### Motor Control
```c
typedef struct dshotMotor_s {
    dshotTelemetry_t dshot_telemtry;  // Decoded feedback
    dshotControl_t   dshot_control;   // Throttle + telemetry request
    dma_ctrl_t       dma;              // DMA channel config
    pwm_ctrl_t       pwm;              // PWM output config
    uint8_t          motor_id;         // 0-3
} dshotMotor_t;

typedef struct dshotSystem_s {
    uart_ctrl_t      telemetry_uart;   // ESC telemetry UART
    dshotMotor_t     motor0;
    dshotMotor_t     motor1;
    dshotMotor_t     motor2;
    dshotMotor_t     motor3;
} dshotSystem_t;
```

### Radio Control
```c
typedef struct {
    RC_Channel_t CH1;   // Roll (right stick horizontal)
    RC_Channel_t CH2;   // Pitch (right stick vertical)
    RC_Channel_t CH3;   // Throttle (left stick vertical)
    RC_Channel_t CH4;   // Yaw (left stick horizontal)
    RC_Channel_t CH5;   // Mode (switch)
    // ... CH6-CH14 (additional channels)
} fs_ia6b_channels_t;
```

### ESC Telemetry
```c
typedef struct dshotTelemetry_s {
    int8_t temperature_u8;              // Temperature in °C
    uint16_t voltage_cv_u16;            // Voltage (centivolts) = V × 100
    uint16_t current_ca_u16;            // Current (centiamps) = A × 100
    uint16_t consumption_mah_u16;       // Energy consumption (mAh)
    uint16_t erpm_u16;                  // eRPM ÷ 100
    bool valid_b;                       // Last frame valid?
} dshotTelemetry_t;
```

---

## File Organization

```
frdmmcxn947_freertos_drone/
├── main.c                          # Entry point & FreeRTOS task definitions
├── dshot.c/.h                      # DShot motor control & telemetry decoder
├── rc_fsia6B.c/.h                  # FS-iA6B RC receiver parser
├── uart_driver_mcxn947.c/.h        # LPUART + DMA abstraction layer
├── pwm_driver_mcxn947.c/.h         # PWM/eTPU control
├── dma_driver_mcxn947.c/.h         # eDMA channel management
├── timer_driver_mcxn947.c/.h       # LPTMR task scheduling
│
├── CMakeLists.txt                  # Build configuration (ZEPHYR/NXP SDK)
├── CMakePresets.json               # Build presets (debug, release)
├── prj.conf                        # Zephyr kernel config
├── Kconfig                         # Config options
│
├── frdmmcxn947_cm33_core0/          # Board-specific configs
└── debug/                           # Build artifacts & CMake cache
```

---

## Prerequisites
- **NXP MCUXpresso IDE** v12.0+ or **VS Code** with CMake extension
- **FreeRTOS** kernel (included via MCUX SDK)
- **MCUXpresso SDK** for MCXN947
- **Zephyr RTOS** build system (configured in CMakeLists.txt)

---

## Performance Characteristics

### Task Scheduling
- **Timer Frequency:** 800 Hz (1.25 ms period)
- **Motor Control Rate:** 800 Hz (4 motors × 200 Hz effective per motor)
- **RC Parser Rate:** Event-driven (on DMA completion, typically 50-100 Hz)
- **ESC Telemetry Rate:** Event-driven (on DMA completion, ~200 Hz total)

### Latency
- **RC → Motor Command:** < 3 ms (next timer tick)
- **Motor Telemetry → Available:** 1ms (ESC response time)
- **ISR to Task Switch:** < 100 µs (FreeRTOS)

---

## Next Steps

### Immediate (Sensor Integration)
1. Design LPSPI driver for IMU/barometer (DMA-enabled)
2. Implement sensor initialization & data reading
3. Create interrupt/notification handlers for fast sampling

### Short-term (Sensor Fusion)
1. Choose fusion algorithm (complementary filter or EKF)
2. Implement SensorFusionTask (runs at 100-200 Hz)
3. Output attitude (roll, pitch, yaw) and altitude

### Medium-term (Control Loop)
1. Implement PID controller for attitude stabilization
   - Inner loop: gyro rate control (1000+ Hz possible)
   - Outer loop: attitude from accelerometer/complementary filter (200 Hz)
2. Create ControlAlgorithmTask with priority above sensors
3. Add safety checks (failsafe, throttle limits)

### Long-term (Advanced)
- [ ] Adaptive control (gain scheduling)
- [ ] Advanced filtering (Kalman filter)
- [ ] Multi-rate control (fast gyro, slow altitude)
- [ ] Wireless telemetry feedback to ground station
- [ ] Autonomous flight modes (hold altitude, GPS waypoints)

---

## References & Standards

- **DShot Protocol:** [Betaflight Dshot Spec](https://betaflight.com/docs/development/API/Dshot)
- **FS-iA6B Receiver:** FlySky iBus protocol
- **FreeRTOS:** [Kernel documentation](https://www.freertos.org/)
- **NXP MCXN947:** [Reference Manual](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXN947)
- **eDMA:** Enhanced Direct Memory Access for real-time I/O

---

## Author & License

**Author:** Diego  
**Created:** April 2026  
**Development Status:** Alpha (flight-critical systems pending)

---

## Safety & Disclaimer

⚠️ **WARNING:** This flight controller is under active development. **Do NOT use on actual aircraft without thorough testing and failsafe verification.** Ensure proper:
- PWM signal integrity (scope-verified)
- ESC response verification
- RC link failsafe behavior
- Motor rotation direction testing (on bench, without props)
- Emergency stop/disarm procedures

---
