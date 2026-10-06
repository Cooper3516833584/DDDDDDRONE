# FleetBus task-layer integration

The airborne HC-14 is connected to the flight controller's UART2. `AirFleetNode`
uses the existing FC wireless bridge while FleetBus mode is active; it does not
open a CH340 device on the airborne Linux computer.

`attach_air_fleet_node()` creates and starts `FCWirelessTransport` by default.
The flight-controller firmware adds and removes the `BB 33 | length` envelope;
FleetBus receives and sends only the inner frame. Test rigs with a separate
USB-connected HC-14 may explicitly pass `hc14_port` or `hc14_baudrate` to use
`HC14FleetTransport` instead. Its serial port can also be overridden with
`D_TASK_HC14_PORT` and `D_TASK_HC14_BAUDRATE`.
Mission code consumes `node.command_queue.receive()` in its existing task thread and
decides whether and how an accepted command may call existing navigation logic.
The FleetBus worker itself does not perform flight actions.

`DRONE_START_MISSION (0x23)` is the only non-stop command that the disaster
survey task explicitly enables while its endpoint remains read-only.  It is
queued for the task thread and is allowed before navigation pose freshness is
available; receiving the frame alone never invokes takeoff or another flight
operation.

The disaster survey publishes its field pose in centimetres with the field
bottom-left as `(0,0)`, `+X` to the right and `+Y` upward. Its 3x5 cell centres
are fixed and shared with the ground station, so the task omits the optional
absolute-position extension to keep every HC-14 response bounded. An
unrecognized cell remains `TerrainCode.UNKNOWN (0)` even when the survey is
marked complete.
