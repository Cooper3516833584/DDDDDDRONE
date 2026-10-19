# Testing Guide

> Status: documentation only. **No test was executed** for this change. This
> repository has no automated test framework at the root; validation is mostly
> static checks plus on-vehicle probes.

## Test classes

| Class | Purpose | Where | Safe without hardware? |
|---|---|---|---|
| Static checks | Syntax, diff hygiene | whole repo | Yes |
| Pure-logic scripts | Frame packing/parsing, CRC, coordinate conversion | `python_sdk/testcode/`, ad-hoc | Yes if imports do not open serial ports |
| Hardware probe | FC serial, radar, servos, screen | `python_sdk/testcode/` | **No** |
| Mission scripts | Drop, navigation, vision tasks | `python_sdk/*.py` | **No — can arm/fly** |

## Documented checks

```bash
python3 -m py_compile <changed file>
git diff --check
git status --short
git diff --stat
```

`AGENTS.md` §12 defines these as the default static validation. It explicitly does
**not** require pytest and forbids inventing a framework for appearance.

## What was not run here

- Any `python_sdk` script: **NOT RUN** — they expect the FC serial device and may
  arm or move the aircraft.
- Keil firmware build: **NOT RUN** (no toolchain on this host).
- Radar/FC probes: **NOT RUN** by policy.

## Rules

- Pure-logic scripts must not open `/dev/ttyACM*`, `/dev/ttyUSB*` or the radio.
- Do not add test dependencies that break the ARM64 target.
- Never present a syntax check as a flight test; record real runs in `details.md`
  or a dated report with conditions.
