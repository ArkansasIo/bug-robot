# Testing Guide

## Current test inventory

- `tests/test_gait.asm`: gait-offset and phase test source.
- `tests/test_math.asm`: fixed-point/math test source.

These files are assembly test routines; the repository does not yet provide a complete standalone test runner. Their presence alone does not mean the tests have been executed.

## Build validation

From the repository root:

```sh
make -C build clean all
```

If tools are not installed, install NASM and GNU binutils for your platform first. Record tool versions and the full build log when reporting failures.

## Test design recommendations

- Verify all gait types and every leg/segment boundary.
- Test phase increment values of 0, 1, 127, 128, 255 and wraparound.
- Test invalid gait IDs and segment counts outside the supported range.
- Exercise fixed-point multiply/divide with zero, signed values, saturation, and overflow boundaries.
- Test packet CRC and length validation with valid, truncated, malformed, and corrupted frames.
- Stub physical I/O during automated tests. Never issue servo output from a unit test.

## Future CI

A safe CI pipeline should assemble each module, link the firmware, run host-side pure-algorithm tests, and upload logs on failure. Hardware tests must be a separate, manually approved workflow and must never run automatically against physical actuators.
