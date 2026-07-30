# Local-to-FIELD Coordinate Contract

## Status and authority

This document fixes the coordinate contract for the three-end D-task
localization integration. It is the implementation authority for later phases.
Phase 01 adds no runtime behavior: no device is commanded, no coordinate-frame
command is sent, and no existing map, navigation, radio, or flight-control
behavior is changed.

Each device reports a pose in its own startup-local frame. The ground station
is the single authority that converts those poses to the shared `FIELD` frame.
Devices must not pre-apply a FIELD offset or rotation, and must not rebase a
radar map or navigation frame in response to a ground-station coordinate
command.

## FleetBus wire contract

`REPORT` keeps its existing binary layout and has these meanings:

| Field | Meaning |
| --- | --- |
| `x_cm`, `y_cm`, `z_cm` | local-frame position in centimetres |
| `heading_cdeg` | local-frame heading, `0..35999`, top-down counter-clockwise positive |
| `vx_cm_s`, `vy_cm_s`, `vz_cm_s` | local-frame velocity in centimetres per second |

The protocol frame format, CRC, node IDs, polling, turn-around timing, retry
rules, and REPORT binary layout are not changed by this contract.

### Drone local frame

The drone origin is the location passed to `calibrate_basepoint()` for the
current task. Its `+X` and `+Y` remain the existing Navigation local axes.
Navigation's internal `current_yaw` remains clockwise-positive; the FleetBus
heading is `(-current_yaw) % 360`.

For this integration, the task-level attachment must use
`position_transform=None` and `heading_offset_deg=0.0`. It must not start
legacy `AircraftGroundStation`/`GroundStationLink` together with
`AirFleetNode`, because both own the same flight-controller wireless callback.

### Car local frame

The car origin is the rear-axle centre at startup, after
`rebase_calibration_to_start_pose()`. `+X` points along the startup vehicle
heading and `+Y` points to the vehicle's left. Heading is top-down
counter-clockwise positive. A car FleetBus state provider must report
`navigation.pose`, not the radar-native `Pose2D.yaw_cw_deg`.

FleetBus mode must not call `_on_coordinate_frame_command()` or any equivalent
map-rebase path. One process has exactly one HC-14 business-protocol owner.

## FIELD frame and fixed transform

`FIELD` has its origin at the map's lower-left corner, `+X` to the right,
`+Y` upward, centimetre units, and counter-clockwise-positive heading. Field
width and height are configuration values. The provisional D-task extent may be
`400 x 500 cm`, but the H point, car rear-axle start, and initial vehicle
heading must come from the site plan or measurement; they are deliberately not
invented here.

For each node, the ground station stores `origin_world_x_cm`,
`origin_world_y_cm`, and `local_x_heading_world_deg`. The latter is the
counter-clockwise angle from FIELD `+X` to the node's local `+X`.

With `theta = radians(local_x_heading_world_deg)`, local to FIELD is:

```text
x_world = origin_x + cos(theta) * x_local - sin(theta) * y_local
y_world = origin_y + sin(theta) * x_local + cos(theta) * y_local
heading_world = (heading_local + local_x_heading_world_deg) % 360
```

FIELD to local is the inverse and is mandatory for map-click targets:

```text
dx = x_world - origin_x
dy = y_world - origin_y
x_local =  cos(theta) * dx + sin(theta) * dy
y_local = -sin(theta) * dx + cos(theta) * dy
heading_local = (heading_world - local_x_heading_world_deg) % 360
```

Velocity is rotated by the same rotation but never translated. Thus the only
allowed direction of pose presentation is `REPORT local -> FIELD -> map`; the
only allowed direction of a clicked goal is `FIELD -> local -> command`.

## Reference point versus display geometry

`world_pose` always denotes the native localization reference point: the
existing Navigation reference for the drone and the rear-axle centre for the
car. UI-only display offsets must never change `world_pose`, trajectories, or
outgoing commands. The car body-centre default is `7.125 cm` forward of its
rear axle; the drone display offset defaults to zero until measured.

## Protected boundaries

- Drone: do not modify `python_sdk/FlightController/**`, radar localization,
  PID, flight protocol, real-time control, or Navigation coordinate semantics.
- Car: do not modify radar ICP/wall fusion/mount semantics, Navigation's rear
  axle origin/planner/controller/safe stop, or motor/servo/Ackermann hardware
  protocol and calibration.
- Ground station: retain FleetBus wire behavior and disaster-survey features;
  do not place passwords, HMAC material, or keys in JSON configuration.

## Implementation audit at Phase 01 stop

The drone's `NavigationAirStateProvider` already converts Navigation yaw with
the required sign and has a `None` position-transform path. Existing
task-specific documentation that describes a previously published field pose is
not authorization to reuse that absolute-pose behavior for this D-task
integration.

The ground-station `FleetStore` currently copies `REPORT.x_cm/y_cm` directly
into `world_pose`, trajectories, and the map-facing state. It has no
local-to-FIELD transform yet. That work begins in Phase 02; it is intentionally
not implemented by this documentation change.

The inspected car branch already contains FleetBus-related modules, including a
`SET_COORDINATE_FRAME` callback. Its presence is not conformance with this
contract. Phases 04-05 must adapt the current branch so the FleetBus runtime
reports the specified local Navigation pose and never invokes the map-rebase
callback.

## Phase 02 entry conditions

Implement the transform as a ground-station-only pure component, derive FIELD
`world_pose` and trajectories in the store, and preserve raw report data for
diagnostics. Add fake-only tests for the forward/inverse transform and do not
change device communication in that phase.
