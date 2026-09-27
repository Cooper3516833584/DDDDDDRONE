# MID360S ROS2 localization

This workspace uses the pinned `Livox-SDK/livox_ros_driver2` and
`Ericsii/FAST_LIO_ROS2` (`ros2` branch) sources. The setup script installs
the official MID360S launch/config, the project network values, and the
FAST-LIO high-rate odometry patch. The UAV ROS1 repository was used only as
an algorithm reference.

## Prepare and build on Ubuntu 22.04 with ROS2 Humble

Install Livox-SDK2 as required by `livox_ros_driver2` (`liblivox_lidar_sdk_shared.so`
and its headers), ROS2 Humble, `python3-colcon-common-extensions`, and `rosdep`.
From this directory:

```bash
source /opt/ros/humble/setup.bash
bash setup_localization_sources.sh
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

The flight computer (Intel N97) now runs Ubuntu 22.04 with ROS2 Humble. The
Foxy installation it shipped with was upgraded in place, and both localization
packages build on Humble with the command above without changing the pinned
patch. `ros2-apt-source` supplies the Humble repository and its signing key.

The driver source is `src/livox_ros_driver2` at
`21445540f0d100dc86a7e6df312dd70bbdb4afdf`. FAST-LIO is
`src/FAST_LIO_ROS2` at `2fffc570a25d0df172720bac034fbdb6a13d2162`.
The project patch is `patches/fast_lio_highrate.patch`. Rerunning setup does
not apply it twice; the setup script also reverses the recorded earlier patch
before applying a newer version, and can migrate the original project patch.
If unrelated local source edits conflict with an upgrade, setup stops.

The host NIC must use `192.168.1.50/24`; the MID360S address is
`192.168.1.194`. The official MID360S JSON retains its original ports and
the official launch uses `xfer_format=1` for Livox `CustomMsg`.

## Bench operation

With the LiDAR rigidly mounted on a bench, start the driver:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

The bench YAML uses zero/identity *estimator seeds* and online extrinsic
estimation. It is never a flight configuration:

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360s_bench_extrinsic_estimation.yaml rviz:=false
ros2 topic hz /Odometry
ros2 topic hz /Odometry_highrate
ros2 topic hz /LioHealth
```

Record stationary drift, hand translation/rotation direction, monotonic
high-rate timestamps, and the five actual topic rates. Exercise LiDAR loss,
FAST-LIO restart, and stale odometry with propellers removed.

## Flight computer checks

The initial checks below were made before the sensor link was available.
The later sensor-on rates and raw timestamp measurements are recorded in
`../docs/mid360s_c1_progress.md`; offline LIO replay results are in
`../docs/mid360s_c2_progress.md`, `../docs/mid360s_c3_progress.md`, and
`../docs/mid360s_c4_c5_progress.md`.

- `livox_ros_driver2` and `fast_lio` build on Ubuntu 22.04 with Humble.
- Both nodes register: `/livox_lidar_publisher` and `/laser_mapping`.
- `/livox/lidar` carries `livox_ros_driver2/msg/CustomMsg`, and
  `laser_mapping` subscribes to `/livox/lidar` and `/livox/imu`.
- `/Odometry` and `/Odometry_highrate` both have a `laser_mapping` publisher.
- The driver logs `bind failed` and `Failed to init livox lidar sdk` when no
  process holds `192.168.1.50` on `enp3s0`. With NetworkManager applying the
  saved static address, the same launch reaches `Init lds lidar success!`.
- No `/livox/lidar` or `/livox/imu` publisher exists until the LiDAR actually
  delivers data, so the four topics only carry a `laser_mapping` subscription
  until the MID360S is connected.

Dynamic drift, motion direction, and sensor loss/restart on the flight computer
remain unmeasured. The offline replay rates do not establish live delay or
navigation readiness.

### Live MID360S check (2026-09-27)

The MID360S was attached to the flight computer and checked over SSH. The
Ethernet carrier was present (`enp3s0`, `192.168.1.50/24`), and the sensor at
`192.168.1.194` answered all three pings (0% loss, 0.9–1.9 ms). A short run of
the pinned Livox driver found the sensor, switched it to Normal mode, enabled
its IMU, and created the expected ROS publishers. `/livox/lidar` carried
`livox_ros_driver2/msg/CustomMsg` at approximately 9.98 Hz, matching the
configured 10 Hz. `/livox/imu` was visible in the ROS graph, but a four-second
subscription received no messages. FAST-LIO was not started, so neither
odometry topic was tested. The short driver run was stopped after sampling;
SDK cleanup completed, but the launch reported process exit code `-7` during
shutdown, which remains to be investigated.

The deployed checkout is `/home/fc/dddddrone` (five `d` characters before
`rone`), and its ROS workspace is `/home/fc/dddddrone/ros2_ws`. This spelling
matches the live host and the other deployment records.

## Production coordinate setup

`T_A_B` maps coordinates from frame B into frame A. The fixed manufacturer
LiDAR-to-IMU transform is `T_I_L`: `t_I_L=[-0.011,-0.02329,0.04412] m`,
`R_I_L=I`. The production FAST-LIO template contains these values with
`extrinsic_est_en: false`. `setup_localization_sources.sh` installs that
template if no custom `mid360s_drone.yaml` exists.

The installed LiDAR axes match the aircraft axes: +X forward, +Y left, +Z up.
The LiDAR is on the aircraft centreline, above the body origin. For recording
and diagnostics, an absent `python_sdk/config/mid360s_mount.json` leaves the
bridge at the LiDAR origin (`reference_frame: lidar`). This fallback cannot
authorize production Navigation. Production Navigation requires a measured,
reviewed body mount JSON with `"reviewed": true`, using either explicit `T_I_B`
or `radar_height_above_body_origin_m`. For the height form, the aligned axes
give `t_I_B=t_I_L+[0,0,-height]`. Conflicting or invalid mount entries fail
closed. `ros_boot` can still start the driver and FAST-LIO for diagnostics;
`mis_boot` requires the reviewed mount. Basepoint calibration establishes a
local frame at that body reference point; it does not reset FAST-LIO or apply
a field transform. The height or full `T_I_B` must be measured before mission
startup; this repository does not supply a production value.

`correction_seq` and `anchor_stamp` advance only on quality-valid laser
corrections. A bad scan immediately marks health DEGRADED without refreshing
the valid anchor. After 250 ms without a valid correction, health becomes
LOST. DEGRADED pauses Navigation while retaining its base; recovery requires
10 distinct valid corrections. LOST clears the base and latches until a
disarmed ground reset, after which a new 2 s stationary window is required.

The bridge exposes `set_ceiling_clearance_estimator()` and
`estimate_ceiling_clearance_m(points)` for optional LiDAR-to-ceiling distance.
No estimator is registered by default, and no point-cloud subscription or
background computation is started for this interface. A caller must supply
an estimator and explicitly request a result.

Before flight, validate actual pose direction, stationary drift, timestamps,
topic rates, dynamic motion, and failure handling on the assembled aircraft.
The current zero `common.time_offset_lidar_to_imu` also needs validation.

Only after all bench gates pass, the runtime launch pair is:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360s_drone.yaml rviz:=false
```

`server_ros.py` uses the same launch pair through the existing `RosManager`.
It does not start RealSense, Cartographer, or LD06 localization. The aircraft
Navigation subscribes to `/Odometry_highrate` through the existing rclpy
executor. **Do not use a propeller-on closed loop until the assembled aircraft
passes the remaining bench and dynamic checks.**
