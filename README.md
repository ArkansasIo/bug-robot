# Centipede Robot — x86 Assembly Firmware

A modular NASM assembly firmware prototype for a multi-segment centipede robot. It contains gait generation, kinematics, servo-control abstractions, sensor/communications modules, system state handling, and low-level build scaffolding.

> **Hardware and execution warning:** This repository is a firmware prototype, not a verified robot controller. The current build targets 32-bit ELF and uses low-level I/O and linker assumptions. Do not connect it to actuators or run it on real hardware until the target platform, interrupt setup, memory map, drivers, and emergency-stop path have been reviewed and tested. Run software tests and simulation first.

## Repository layout

- `src/` — assembly modules for startup, control, gait, kinematics, sensors, servo control, communications, state machine, and diagnostics.
- `inc/` — shared macros and data structures.
- `config/` — robot and platform constants.
- `linker/` — linker script.
- `build/` — Makefile and Windows build script.
- `tests/` — assembly-level gait and math test sources.
- `docs/` — architecture and hardware-interface notes.

## Requirements

- NASM 2.15 or newer
- GNU binutils (`ld`, `objcopy`, `size`)
- GNU Make for the Makefile workflow
- Optional: QEMU and GDB for emulation/debugging (target support depends on the linked image format)

## Build

From the repository root on Linux/WSL:

```sh
make -C build
```

Clean generated files:

```sh
make -C build clean
```

On Windows, use a shell with NASM and a compatible 32-bit linker installed:

```bat
cd build
build.bat
```

The build scripts compile sources; successful linking alone does not prove the binary is bootable on a particular board or safe for hardware operation.

## Development workflow

1. Review `docs/architecture.md` and `docs/hardware_interface.md`.
2. Build with NASM and inspect all warnings/errors.
3. Add host-side or emulator-based tests for pure algorithms (math, gait phase, packet encoding).
4. Verify memory layout and CPU execution mode against the intended target.
5. Test the stop path and actuator-disable behavior with outputs disconnected.
6. Validate sensor and communications drivers against the actual hardware datasheets before connecting the robot.

## Safety principles

- The emergency-stop circuit should be independent of the main CPU and software loop.
- On sensor failure, communication loss, invalid commands, or watchdog timeout, outputs should transition to a hardware-defined safe state.
- Servo limits, current limits, battery undervoltage, and thermal protection should be enforced at the driver/power stage as well as in firmware.
- Never test unverified motion code with the robot supported only by its own legs.

## Contributing

Keep changes modular, document hardware assumptions, and include a test or reproducible validation procedure for each behavior change. See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT — see [LICENSE](LICENSE).
