# Centipede Robot - x86 Assembly Project

A modern, modular centipede robot control system written entirely in x86 assembly (NASM syntax). This project demonstrates advanced assembly programming techniques including modular architecture, interrupt-driven control, fixed-point mathematics, and multi-segment kinematics.

## Architecture

```
                    ┌─────────────────────────────────┐
                    │        Main Controller           │
                    │  (Gait Engine + State Machine)   │
                    └─────────────────────────────────┘
                                │
                    ┌───────────┴───────────┐
                    │    Communication Bus  │
                    │    (I2C / Serial)     │
                    └───────────┬───────────┘
                    ┌───────────┼───────────┐
                    │           │           │
              ┌─────▼────┐ ┌───▼───┐ ┌─────▼────┐
              │ Segment 1│ │Seg. 2 │...│ Segment N│
              │ (Head)   │ │       │   │ (Tail)   │
              └──────────┘ └───────┘ └──────────┘
```

## Key Features

- **Modular Segments**: Each body segment runs identical control code
- **Kinematics Engine**: Real-time inverse kinematics for leg positioning
- **Gait Patterns**: Multiple gaits (wave, ripple, alternating tripod)
- **Sensor Fusion**: Collision detection, terrain sensing, balance
- **Interrupt-Driven**: Timer-based control loops
- **Fixed-Point Math**: High-precision calculations without FPU
- **State Machine**: Hierarchical behavior management

## Build Requirements

- NASM (Netwide Assembler) 2.15+
- GCC or LD (for linking)
- Windows (with build.bat) or Linux (with Makefile)

## Build Instructions

### Windows
```bash
cd build
build.bat
```

### Linux
```bash
make -f Makefile
```

## Project Structure

```
centipede_robot/
├── src/           # Assembly source files
├── inc/           # Include files (macros, equates, structs)
├── config/        # Configuration constants
├── linker/        # Linker scripts
├── build/         # Build output
├── docs/          # Documentation
└── tests/         # Unit tests
```

## Gait Patterns

1. **Wave Gait** - Sequential leg movement from tail to head (slow, stable)
2. **Ripple Gait** - Continuous wave propagation (medium speed)
3. **Alternating Tripod** - Three legs supporting while three move (fast)

## License

MIT License - See LICENSE file for details

