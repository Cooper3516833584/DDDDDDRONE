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

## Production measurements required

After independent 6DoF bench estimates of MID360S LiDAR-to-IMU `extrinsic_T`
and `extrinsic_R` agree, fill `config/mid360s_drone.yaml` from the checked-in
template, set `extrinsic_est_en: false`, and rerun setup. Measure the installed
IMU-to-aircraft-body transform `T_I_B`, then fill
`../python_sdk/config/mid360s_mount.json` from its template. These two actual
files are deliberately ignored by Git until their measured values are
reviewed. `server_ros.py` rejects production FAST-LIO and mission startup
while either measurement is absent.

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
