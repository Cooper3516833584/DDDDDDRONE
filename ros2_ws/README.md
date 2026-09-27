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
not apply it twice.

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
```

Record stationary drift, hand translation/rotation direction, monotonic
high-rate timestamps, and the four actual topic rates. Exercise LiDAR loss,
FAST-LIO restart, and stale odometry with propellers removed.

## Verified on the flight computer

Checked on the upgraded flight computer with no LiDAR link, so these are
launch-time facts only, not rate or drift measurements:

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

The following remain unmeasured: IMU message rate and continuity, FAST-LIO
odometry rates, stationary drift, motion direction, and behavior after sensor
loss/restart.

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

## Production measurements required

Two spatial transforms must be measured and reviewed before production
localization can start. In the notation below, `T_A_B` maps coordinates from
frame B into frame A.

| Transform | Meaning and storage | How to measure |
| --- | --- | --- |
| `T_I_L` | Pose of the LiDAR frame L in the built-in IMU frame I. FAST-LIO's `mapping.extrinsic_T` is `t_I_L` in metres and `mapping.extrinsic_R` is `R_I_L` as nine row-major values, satisfying `p_I = R_I_L p_L + t_I_L`. | Rigidly fixture the sensor on a bench with propellers removed. Once IMU messages are flowing, use `mid360s_bench_extrinsic_estimation.yaml` (`extrinsic_est_en: true`) and move the whole fixture through varied translations and rotations about all three axes. Let the estimate settle; record the patch's `BENCH LiDAR-to-IMU` output across several runs and compare the resulting transforms. Do not use the zero/identity seed as a measurement. |
| `T_I_B` | Pose of the aircraft body frame B in IMU frame I. In `mid360s_mount.json`, `translation_m` is the body-origin position expressed in I, and `quaternion_xyzw` represents `R_I_B`. The runtime computes `T_WB = T_WI T_I_B`. | Define the project body origin and axes first. Survey the installed LiDAR frame L relative to that body datum as `T_L_B`, using the official Livox frame origin/axis definition and measured mounting geometry; compose it with the measured FAST-LIO transform: `T_I_B = T_I_L T_L_B`. If manufacturer data identifies the IMU origin/axes directly, survey `T_I_B` from that reference instead. A survey that gives IMU pose in body coordinates (`T_B_I`) must be inverted: `R_I_B = R_B_I^T`, `t_I_B = -R_B_I^T t_B_I`. Store translation in metres and a normalized x-y-z-w quaternion. Do not substitute the task startup/basepoint for the rigid body datum. |

For `T_I_L`, compare repeated estimates from different motions and, where
available, an independent geometric/reference measurement. For `T_I_B`, record
the body datum, axis convention, measured dimensions, and any inverse used.
After filling the ignored production files, set
`extrinsic_est_en: false`, rerun the setup overlay, and validate pose direction,
stationary drift, timestamps, and all four topic rates. The live check above
did not measure either transform; the missing IMU messages also block FAST-LIO's
online `T_I_L` estimate. `T_I_B` can be surveyed mechanically once the body
datum and sensor-frame reference are established.
`common.time_offset_lidar_to_imu` is a separate temporal offset, not a spatial
extrinsic; the current zero value also needs validation. `server_ros.py`
rejects production FAST-LIO and mission startup while required measurement
files or values are absent.

Only after all bench gates pass, the runtime launch pair is:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360s_drone.yaml rviz:=false
```

`server_ros.py` uses the same launch pair through the existing `RosManager`.
It does not start RealSense, Cartographer, or LD06 localization. The aircraft
Navigation subscribes to `/Odometry_highrate` through the existing rclpy
executor. **Do not use a propeller-on closed loop until both transforms and
the remaining bench checks are measured and passed.**
