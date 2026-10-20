# Contributing

Thanks for helping improve the Centipede Robot firmware.

## Before making changes

- Keep NASM source compatible with the repository's declared output format unless a target migration is discussed.
- Document CPU mode, memory-map assumptions, I/O port use, and hardware dependencies.
- Prefer small changes with one responsibility.
- Do not treat a successful assembly/link as evidence of hardware safety.

## Validation checklist

- [ ] Run `make -C build clean all` with NASM and binutils installed.
- [ ] Inspect assembler/linker diagnostics.
- [ ] Run available unit tests or add a reproducible test command.
- [ ] Verify fixed-point boundary cases and phase wraparound.
- [ ] Check interrupt and register-preservation conventions for modified routines.
- [ ] Confirm emergency-stop and output-disable behavior before any actuator test.
- [ ] Update relevant documentation.

## Hardware safety

Test with actuators disconnected or power-disabled first. Use a hardware emergency stop independent of the firmware loop. Bench testing should be supervised and should follow the specifications of the actual servos, drivers, battery, and controller board.
