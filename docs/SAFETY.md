# Safety and Hardware Validation

This repository provides experimental low-level robot firmware. It has not been independently safety-certified.

## Before connecting actuators

1. Identify the exact CPU, board, boot mode, timer, interrupt controller, UART/I2C peripherals, and memory map.
2. Confirm the linker output and entry point match that target.
3. Replace example I/O port addresses with documented, board-specific interfaces.
4. Verify servo pulse ranges, mechanical travel limits, torque/current limits, and power supply ratings.
5. Test initialization failure, sensor disconnect, bus timeout, watchdog, and emergency-stop states.
6. Ensure actuator power can be cut independently of software.
7. Confirm that boot failures or crashes leave outputs disabled.

## Required fault behavior

- Default output state is disabled until initialization and health checks succeed.
- A stale sensor sample must not be interpreted as valid data.
- A communications timeout must stop or safely hold motion according to a validated policy.
- Invalid gait, segment IDs, or servo indices must be rejected.
- A watchdog reset must not automatically restart motion without a deliberate re-arm procedure.

## Test sequence

1. Static inspection and assembly/link validation.
2. Algorithm tests with synthetic values.
3. Simulator or emulator tests with hardware I/O stubbed.
4. Bench tests with actuator power disconnected.
5. Single-actuator tests with current limits and mechanical stops.
6. Incremental multi-actuator tests in a physically restrained workspace.

Do not rely on software alone for personnel protection. Use a physical emergency stop and appropriate guarding.
