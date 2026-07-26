# Centipede Robot Architecture

## System Overview

The centipede robot control system is implemented entirely in x86 assembly language (NASM syntax). It manages a modular, multi-segment robot with up to 16 body segments, each with 2 legs (3 degrees of freedom per leg), totaling 96 servo motors.

## Architecture Layers

```
┌──────────────────────────────────────────────────────────┐
│                     State Machine                        │
│              (Behavior Decision Layer)                    │
├──────────────────────────────────────────────────────────┤
│                    Gait Engine                           │
│              (Leg Trajectory Generation)                  │
├──────────────────────────────────────────────────────────┤
│                 Kinematics Engine                        │
│              (IK/FK Calculations)                         │
├──────────────────────────────────────────────────────────┤
│               Servo Control Layer                        │
│              (PWM Generation)                             │
├──────────────────────────────────────────────────────────┤
│              Communication Layer                         │
│         (UART, I2C, Inter-segment Bus)                    │
├──────────────────────────────────────────────────────────┤
│               Sensor Fusion Layer                        │
│         (IMU, Ultrasonic, Force Sensors)                  │
├──────────────────────────────────────────────────────────┤
│               HAL / Hardware Drivers                     │
│         (Timer, Interrupts, GPIO, I2C, ADC)               │
└──────────────────────────────────────────────────────────┘
```

## Module Responsibilities

### 1. **main.asm** - System Entry & Main Loop
- Stack initialization
- Module initialization sequence
- 100Hz control loop (sensor → state machine → gait → servo → comm)
- Timer interrupt configuration (PIT at 1kHz)
- Emergency stop handling

### 2. **init.asm** - System Initialization
- Hardware detection (segment enumeration via I2C)
- Memory initialization (BSS clearing)
- PIC remapping and interrupt configuration
- Subsystem initialization in proper order

### 3. **state_machine.asm** - Behavior Control
- Hierarchical state machine with 6 states
- State transitions based on commands and sensor data
- Behavior adaptation to terrain types
- Idle timeout and safety checks

### 4. **gait_engine.asm** - Gait Patterns
- Three gaits: Wave, Ripple, Alternating Tripod
- Phase calculation and leg offset generation
- Trajectory computation (foot position over time)
- Stance/swing phase management

### 5. **kinematics.asm** - Inverse Kinematics
- 3-DOF leg IK (coxa, femur, tibia angles from foot position)
- Forward kinematics for validation
- Coordinate transformation and body adjustment
- Fixed-point math library (CORDIC-based)

### 6. **servo_control.asm** - PWM Management
- 96-channel PWM generation via PCA9685 controllers
- Double-buffered servo updates (glitch-free)
- Servo calibration (min/max/Center)
- Servo disabling for safe states

### 7. **sensors.asm** - Sensor Processing
- MPU6050 IMU driver (accelerometer + gyroscope)
- HC-SR04 ultrasonic distance sensors
- Force sensing resistor array
- Battery voltage monitoring
- Complementary filter for orientation estimation
- Collision detection logic

### 8. **communication.asm** - Data Exchange
- UART for external serial commands (115200 baud)
- I2C for segment-to-segment communication (400kHz)
- Byte-stuffed packet protocol with CRC8
- Master/slave addressing scheme

### 9. **debug.asm** - Diagnostics
- Real-time status display via serial console
- Hex and decimal print utilities
- Circular error log buffer
- Performance profiling (loop timing)
- System state dump

## Data Flow

```
Timer Interrupt (1kHz)
    │
    ▼
Set tick_flag ──────────────────────────────────────────────┐
                                                             │
Main Loop (100Hz)                                            │
    │                                                        │
    ├── Wait for tick_flag ───────────────────────────────────┘
    │
    ├── sensor_update()         ← Read IMU, Distance, Force
    │       │
    │       └── fuse_imu_data() ← Complementary filter
    │
    ├── state_machine_update()  ← Evaluate transitions
    │       │
    │       └── check_transition_conditions()
    │
    ├── gait_engine_run()       ← Generate leg trajectories
    │       │
    │       ├── advance_gait_phase()
    │       ├── generate_gait_offsets()
    │       └── For each leg:
    │               ├── calculate_leg_trajectory()
    │               └── calculate_ik() → leg_set_position()
    │
    ├── servo_control_loop()    ← Update PWM outputs
    │
    └── communication_update()  ← Handle I/O
```

## Memory Map

| Address  | Size  | Content          |
|----------|-------|------------------|
| 0x1000   | 32KB  | Code (.text)     |
| 0x9000   | 8KB   | RAM              |
| 0x9000   |       | .data section    |
| 0x9200   | 1KB   | .bss section     |
| 0x9600   | 1KB   | Stack            |
| 0x9A00   | 1KB   | Heap             |

## Fixed-Point Format

All real numbers use Q16.16 fixed-point format:
- 32-bit signed integer
- Upper 16 bits: integer part
- Lower 16 bits: fractional part
- Range: ±32767.9999
- Resolution: 1/65536 ≈ 0.000015

## Interrupt Map

| IRQ  | INT  | Handler           |
|------|------|--------------------|
| 0    | 20h  | Timer tick         |
| 1    | 21h  | Keyboard (future)  |
| 4    | 24h  | Serial (UART)      |
| 5-7  | 25h  | Reserved           |

## Communication Protocol

### UART Packet Format
```
┌────┬────┬────┬────┬──────┬──────────┬────┬────┐
│ AA │Type│Src │Dst │ Len  │ Payload  │CRC │ 55 │
│ 1B │ 1B │ 1B │ 1B │  2B  │ 0-64 B  │ 1B │ 1B │
└────┴────┴────┴────┴──────┴──────────┴────┴────┘
```

### I2C Segment Addressing
- Master: 0x00
- Segments: 0x30-0x3F (up to 16)
- Broadcast: 0xFF

## Gait Timing

```
Wave Gait (8 segments, 16 legs):
        Leg 0  ████░░░░░░░░░░░░░░░░
        Leg 1  ░░████░░░░░░░░░░░░░░
        Leg 2  ░░░░████░░░░░░░░░░░░
        ...    ░░░░░░░░░░░░░░░░░░░░
        Leg 15 ░░░░░░░░░░░░░░████░░
        └─────────────────────────►
              256 phase units

      ██ = Stance (60%)  ░░ = Swing (40%)
```

## Performance Targets

| Metric           | Target    |
|------------------|-----------|
| Control Loop     | 100 Hz    |
| Servo Update     | 50 Hz     |
| IMU Read         | 100 Hz    |
| Sensor Update    | 50 Hz     |
| Serial Baud      | 115200    |
| I2C Speed        | 400 kHz   |
| Code Size        | < 32 KB   |
| RAM Usage        | < 8 KB    |
| Cycle Time       | < 10 ms   |

