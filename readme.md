# Lingxiao (ANOTC-based) Flight Controller & Companion SDK

STM32F4 flight-controller firmware for the Lingxiao (凌霄 / 匿名科创) platform,
together with the Python SDK, ROS 2 localization workspace and deployment units
used by the companion computer.

> **Upstream attribution.** `FcSrc/`, `DriversBsp/`, `DriversMcu/` and `HMI/` are
> derived from **ANOTC (匿名科创) official code** and vendor documentation, not from
> scratch. The vendor manual `匿名--凌霄--飞控手册.V1.07pdf.pdf` is included as
> reference. This repository must not be presented as an original flight-controller
> implementation. See [docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md) §1.

## Project at a glance

| Layer | Contents | Origin |
|---|---|---|
| FC firmware | `FcSrc/` (Lingxiao link, scheduler, state, wireless), `DriversBsp/`, `DriversMcu/` | ANOTC upstream + local integration |
| USB stack | `ThirdParty/CherryUSB/` (Apache-2.0), CMSIS-DAP, FatFs | third-party |
| Companion SDK | `python_sdk/FlightController/` (`Base → Protocal → Application`), mission scripts | local work |
| Fleet link | `python_sdk/fleet_bus/` — FleetBus V1 over HC-14 | shared with the ground station |
| ROS 2 | `ros2_ws/` — MID360s + Fast-LIO configuration and patches | local integration (optional) |
| Deployment | `deploy/` — systemd units, udev rules, installer scripts | local work |

## What it does

- Serves as the flight-controller firmware base for the vehicle: sensor and RC
  handling, scheduler, wireless transport, and a binary protocol to the companion.
- Exposes the FC to the companion Linux computer through `python_sdk/FlightController`
  (serial or via `FC_Server`/`FC_Client`).
- Exchanges high-level task commands and acknowledgements with the ground station
  using FleetBus V1 (`payload + CRC16-CCITT-FALSE`, wrapped as `BB 33 | len | frame`).
- Provides optional MID360s LiDAR + Fast-LIO localization for navigation missions.

## Protocol and safety facts

- Wireless path: FC uses command `0x0D` to send and `0x07` for the wireless
  callback; the bridge adds the `BB 33 | bridge_len:u8 | FleetBus frame` envelope.
- FleetBus V1 uses **CRC16 only — no HMAC**. HMAC belongs to the older
  `GroundStationLink` V2 stack in `python_sdk/FlightController/Components/GroundStationLink/`.
- The ground station sends only high-level task commands, never real-time attitude
  or velocity setpoints.
- Unknown, CRC-failed or duplicated commands must not trigger a new flight action;
  STOP is high-priority and idempotent.
- When `server_ros.py` (`FC_Server`) is running, it owns the FC serial port; new
  scripts must use `FC_Client` instead of opening the serial device themselves.
- `details.md` is the authoritative field-troubleshooting reference (device paths,
  radio parameters, verification history). It contains no credentials.

## Repository map

```text
FcSrc/                  flight-controller firmware sources (ANOTC-derived)
DriversBsp/             board support drivers
DriversMcu/STM32F407/   ST standard peripheral library and CMSIS
ThirdParty/CherryUSB/   USB stack (Apache-2.0)
HMI/                    UART screen project assets
python_sdk/             companion Python SDK, missions and tools
python_sdk/fleet_bus/   FleetBus V1 implementation
ros2_ws/                MID360s + Fast-LIO workspace configuration
deploy/                 systemd units, udev rules, install scripts
docs/                   architecture and project documentation
details.md              field notes: ports, radio, protocol, safety boundary
```

## Running and hardware conditions

- The firmware builds in the Keil project (`ANO_LX_FC.uvprojx`); it is not a
  host-runnable simulation.
- Companion scripts expect a Linux host with the FC serial device and optional
  sensors attached. Check `details.md` for current device paths and radio settings
  before connecting.
- Real flights require an operator and the documented safety checks; nothing in
  this repository authorizes an autonomous flight from a cold start.

## Validation and limitations

- `details.md` §5 records logic-level checks (12 onboard + 38 ground-station tests,
  byte-identical frames) and that the wireless bridge was online. Unknown or
  real-flight conditions are explicitly marked **not verified**.
- Firmware internals are upstream-derived; local work is concentrated in the
  Python SDK, task/vision scripts, ROS 2 integration and deployment.
- A flight-log archive is tracked at the repository root; it is not analysed here.
- No root `LICENSE`. Third-party licenses are retained under `ThirdParty/`, and ST
  StdPeriph/CMSIS headers keep their ST copyright.

## Related documents

- [docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md)
- [details.md](details.md) — ports, HC-14 parameters, FleetBus framing, safety boundary
- [python_sdk/STM32_SERVOS.md](python_sdk/STM32_SERVOS.md) — servo notes
- [ros2_ws/README.md](ros2_ws/README.md) — localization workspace
