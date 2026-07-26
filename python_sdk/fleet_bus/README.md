# FleetBus task-layer integration

`AirFleetNode` owns the flight controller's single wireless callback while FleetBus
mode is active. A task must explicitly choose FleetBus mode and must not create or
start the legacy `AircraftGroundStation` / `GroundStationLink` in the same process.

`attach_air_fleet_node()` creates and starts the shared `FCWirelessTransport`.
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
absolute-position extension to keep every FC UT2/HC-14 response bounded. An
unrecognized cell remains `TerrainCode.UNKNOWN (0)` even when the survey is
marked complete.
