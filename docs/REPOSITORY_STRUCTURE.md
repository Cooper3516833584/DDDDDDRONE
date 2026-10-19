# Repository Structure

> Describes the current layout. No production source was moved by this change.

```text
DDDDDDRONE/
├── FcSrc/                  FC firmware (ANOTC-derived): Lingxiao link, scheduler,
│                           state, wireless, USB CDC
├── DriversBsp/             board support drivers
├── DriversMcu/STM32F407/   ST StdPeriph + CMSIS (third-party, keep copyright)
├── ThirdParty/CherryUSB/   USB stack (Apache-2.0) + CMSIS-DAP + FatFs
├── HMI/                    UART-screen project assets
├── python_sdk/
│   ├── FlightController/   FC protocol stack (Base/Protocal/Application),
│   │                       Components, Solutions, Remote
│   ├── fleet_bus/          FleetBus V1 (CRC16) shared with ground station
│   ├── server.py, server_ros.py   FC_Server hosts (single serial owner)
│   ├── rescue_drop_2026*.py, robocup_target_detector.py, fastlio_control.py
│   ├── testcode/           hardware probes and ad-hoc checks
│   └── former_code/        archived code (read-only)
├── ros2_ws/                MID360s + Fast-LIO configuration and patches
├── deploy/                 systemd units, udev rules, install scripts
├── docs/                   architecture, testing, structure
├── details.md              field notes: ports, HC-14, FleetBus, safety boundary
└── readme.md               project entry
```

## Entry points

| Purpose | Where |
|---|---|
| Firmware build | `ANO_LX_FC.uvprojx` (Keil) |
| FC service | `python_sdk/server_ros.py` (`FC_Server`) |
| Mission scripts | `python_sdk/rescue_drop_2026*.py`, `robocup_target_detector.py` |
| Deployment | `deploy/*.service.in`, `deploy/install_*.sh` |

## Where to add code

- FC firmware / protocol → `FcSrc/` (high-risk; read `AGENTS.md` first)
- Companion SDK and missions → `python_sdk/`
- Fleet bus → `python_sdk/fleet_bus/`
- ROS 2 localization → `ros2_ws/`
- Deployment → `deploy/`

`python_sdk/FlightController/**` is a protected layer: modify only for a reproduced
low-level defect, with a minimal patch.

## Generated / ignored

`python_sdk/fc_log/`, `fc_log/`, `__pycache__/`, `.vscode/`, ROS 2 build/install/log
directories, `ros2_ws/config/mid360s_drone.yaml`, `python_sdk/config/mid360s_mount.json`,
and Keil user files (`*.uvguix.*`) are ignored. The tracked
`ANO_LX_FC.uvguix.TZDEZACR` predates that rule and remains tracked until an explicit
decision is made.

## Related documents

- `docs/SYSTEM_ARCHITECTURE.md`
- `details.md`
- `python_sdk/STM32_SERVOS.md`
- `ros2_ws/README.md`
- `docs/TESTING.md`
