# FleetBus task-layer integration

`AirFleetNode` owns the flight controller's single wireless callback while FleetBus
mode is active. A task must explicitly choose FleetBus mode and must not create or
start the legacy `AircraftGroundStation` / `GroundStationLink` in the same process.

`attach_air_fleet_node()` creates and starts the shared `FCWirelessTransport`.
Mission code consumes `node.command_queue.receive()` in its existing task thread and
decides whether and how an accepted command may call existing navigation logic.
The FleetBus worker itself does not perform flight actions.
