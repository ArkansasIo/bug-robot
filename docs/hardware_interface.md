# Hardware Interface Specification

## GPIO Pin Mapping

### Servo Controllers (PCA9685 × 6)
```
Controller 0: I2C 0x40 → Channels 0-15  (Segments 0-2)
Controller 1: I2C 0x41 → Channels 0-15  (Segments 3-5)
Controller 2: I2C 0x42 → Channels 0-15  (Segments 6-8)
Controller 3: I2C 0x43 → Channels 0-15  (Segments 9-11)
Controller 4: I2C 0x44 → Channels 0-15  (Segments 12-13)
Controller 5: I2C 0x45 → Channels 0-15  (Segments 14-15)

Per Servo Channel:
  CH0: Segment N, Leg Left,  Coxa
  CH1: Segment N, Leg Left,  Femur
  CH2: Segment N, Leg Left,  Tibia
  CH3: Segment N, Leg Right, Coxa
  CH4: Segment N, Leg Right, Femur
  CH5: Segment N, Leg Right, Tibia
```

### IMU (MPU6050)
```
I2C Address: 0x68
SCL: Pin A5
SDA: Pin A4
INT:  Pin D2 (optional)
```

### Ultrasonic Sensors (HC-SR04 × 3)
```
Front Sensor:
  Trigger: PB0
  Echo:    PB1

Rear Sensor:
  Trigger: PB2
  Echo:    PB3

Ground Sensor:
  Trigger: PB4
  Echo:    PB5
```

### Force Sensors (ADC × 16)
```
ADC Channels 0-15 via multiplexer
ADC_CS:   PC0
ADC_CLK:  PC1
ADC_DATA: PC2
```

### Communication
```
UART (External):
  TX: PD1
  RX: PD0
  Baud: 115200

I2C (Internal Bus):
  SCL: PC5
  SDA: PC4
  Speed: 400kHz
```

### Status LEDs
```
LED_RUN:    PD6 (Heartbeat)
LED_ERROR:  PD7 (Error indicator)
LED_COMM:   PB6 (Communication activity)
```

### Emergency Stop
```
E-Stop Input: PD3 (Active LOW, pull-up)
```

## Power System

| Rail    | Voltage | Max Current | Notes            |
|---------|---------|-------------|------------------|
| V_SERVO | 6.0V    | 15A         | Servo motors     |
| V_LOGIC | 5.0V    | 2A          | Microcontroller  |
| V_SENSOR| 3.3V    | 500mA       | Sensors          |
| V_BAT   | 11.1V   | 5000mAh     | LiPo battery     |

## Servo Specifications

| Parameter      | Value     | Notes                    |
|----------------|-----------|--------------------------|
| Model          | MG996R    | Standard servo           |
| Operating Volt | 4.8-7.2V  |                          |
| Stall Torque   | 10 kg·cm  | At 6V                    |
| Speed          | 0.17s/60° | At 6V (no load)          |
| Control Signal | 50Hz PWM  | 1-2ms pulse              |
| Resolution     | 4096 steps| 12-bit PWM controller    |
| Range          | 180°      | 0-180 degrees            |

## Connector Pinout (Segment Interconnect)

```
RJ-11 6P4C (Inter-segment bus)
┌─────┐
│ 1 2 │
│ 3 4 │
│ 5 6 │
└─────┘

Pin 1: VCC (5V)
Pin 2: GND
Pin 3: SDA (I2C)
Pin 4: SCL (I2C)
```

## Timing Diagrams

### Servo PWM
```
50Hz Frame (20ms)
┌────────────────────────────────────┐
│        ┌───┐                       │
│        │   │                       │
│        │   │                       │
│        └───┘                       │
│        ←1-2ms→                     │
└────────────────────────────────────┘
```

### Ultrasonic Trigger
```
Trigger: 10μs pulse
┌──┐                                 │
│  │                                 │
│  │                                 │
└──┘                                 │
←10μs→                               │

Echo: Pulse width = distance
                      ┌──────────┐   │
                      │          │   │
                      │          │   │
                      └──────────┘   │
                      ← pulse width →│
```

### I2C Communication
```
START  ADDRESS  R/W  ACK  DATA  ACK  STOP
┌──┐  ┌──┬──┐  ┌┐  ┌┐  ┌──┬┐  ┌┐  ┌──┐
│  │  │  │  │  ││  ││  │  ││  ││  │  │
└──┘  └──┴──┘  └┘  └┘  └──┴┘  └┘  └──┘
SCL ──┴───────┴───┴───┴──────┴───┴───┴──
SDA ─────────────────────────────────────
```

