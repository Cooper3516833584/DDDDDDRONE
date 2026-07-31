# FleetBus task-layer integration

`AirFleetNode` owns the airborne CH340/HC-14 serial device while FleetBus mode is
active. The radio is connected directly to the airborne Linux computer; FleetBus
traffic does not pass through the flight controller, its ACK path, or UART2.

`attach_air_fleet_node()` creates and starts the shared `HC14FleetTransport`.
It uses `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` at 115200 baud by
default. `D_TASK_HC14_PORT` and `D_TASK_HC14_BAUDRATE` may override those values
without changing mission code. The direct transport keeps the same `BB 33 |
length | FleetBus frame` envelope used by the car and ground station.
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
