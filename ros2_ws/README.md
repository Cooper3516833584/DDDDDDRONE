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
closed. Systemd can start the driver and FAST-LIO without a mount for diagnostics;
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

For manually owned bench sessions only, with both systemd units stopped and
no other localization owner, the launch pair is:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360s_drone.yaml rviz:=false
```

`server_ros.py` checks systemd services and actual message flow without launching another chain.
The aircraft
Navigation subscribes to `/Odometry_highrate` through the existing rclpy
executor. **Do not use a propeller-on closed loop until the assembled aircraft
passes the remaining bench and dynamic checks.**

## Production boot

Production localization has one owner: `mid360s-driver.service` and
`mid360s-fastlio.service`. Both run as `fc`, with `HOME=/home/fc`, the ROS
workspace as their working directory, and explicit Humble/overlay environments.
They run the nodes directly via `ros2 run`, without tmux, launch or RViz.
`ros_boot=0/1/2` checks both/driver/FAST-LIO respectively, never uses sudo and
never launches a fallback. `ros_kill` is refused; `ros_log` reads the journal.
Service/topic presence is diagnostic only. Navigation still checks reviewed
mount, fresh exact-stamp data, health and disarmed ground calibration.

Before serving FC clients or registering mission callbacks, `server_ros.py`
performs one bounded passive subscription check in a child process. The child
fully exits so the bridge does not retain a DDS context. With fresh disarmed FC
telemetry and no mission session, proven deleted DDS shared memory may trigger
one recovery through the existing systemd/ownership-audit/official-cleaner
sequence. Other ROS users or unreadable ownership refuse recovery before any
service is stopped. Unknown sensor/health failures are reported without blind
restart. Startup failure keeps the serial bridge available for diagnosis;
every flight task still needs its own new-map and calibration gates. There is
no in-flight watchdog or automatic flight-control resume.

### Preserve DDS memory after SSH logout

The localization units run as ordinary UID 1000 outside login sessions. On
Ubuntu systemd 249, the default `RemoveIPC=yes` can unlink their live POSIX
shared memory after the last login ends. The process stays running, so an
`active` service and visible topics do not prove data reaches a new task.
See `docs/localization_ipc_logout.md` for the reproduced deletion evidence.

After pulling the reviewed code, install the dedicated-computer policy:

```bash
sudo bash deploy/install_localization_ipc_policy.sh
journalctl -b -u systemd-logind -n 10 --no-pager
systemd-analyze cat-config systemd/logind.conf
```

The installer backs up a prior drop-in under `/var/backups/robocup-ipc-*`,
installs `/etc/systemd/logind.conf.d/99-robocup-ipc.conf`, checks the merged
configuration and requests a SIGHUP reload. No FC or localization service is
started/restarted. `RemoveIPC=no` affects all ordinary users on this dedicated
computer; IPC resources are thereafter cleaned by their owning programs or
reboot. This prevents future logout deletion; already unlinked DDS mappings
still require an authorized ground recovery. Do not delete live `/dev/shm`
files manually.

Rollback: restore the previous drop-in with `cp -a` from the printed backup,
or remove this exact drop-in if the backup contains `previously-absent`;
then run `sudo systemctl kill --kill-who=main --signal=HUP systemd-logind.service`.
Do not restart logind or reboot the aircraft as a configuration check.

### Update, rebuild and install

Before deployment, remove propellers, disarm and stop missions. Inspect system
and user services, desktop autostart, cron, `/etc/rc.local`, tmux and node process
owners. Safely stop any previous localization owner; keep T265 disabled.

```bash
cd /home/fc/dddddrone
git status --short                    # stop if dirty or diverged
git pull --ff-only
cd ros2_ws
source /opt/ros/humble/setup.bash
bash setup_localization_sources.sh
bash setup_localization_sources.sh    # idempotency check
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
cd ..
sudo bash deploy/install_mid360s_localization_services.sh
```

Pull alone does not update the ignored source or binaries: apply the overlay
and rebuild. The installer checks configs/overlays, refuses active units or
unmanaged localization nodes, verifies rendered units, backs up previous units
and enable states under `/var/backups/mid360s-localization-*`, installs and
enables. It does not start or restart nodes. Ambiguous repository paths
(spaces or shell metacharacters) are rejected.

```bash
sudo systemctl start mid360s-driver.service
sudo systemctl start mid360s-fastlio.service
systemctl status mid360s-driver.service mid360s-fastlio.service --no-pager
journalctl -b -u mid360s-driver.service --no-pager
journalctl -b -u mid360s-fastlio.service --no-pager
sudo systemctl stop mid360s-fastlio.service
sudo systemctl stop mid360s-driver.service
sudo systemctl disable mid360s-fastlio.service mid360s-driver.service
```

Stopping the driver also stops FAST-LIO through `Requires`; start both after
a driver stop/restart. `Restart=on-failure`, `RestartSec=2`, at most 5 starts
in 30 s, and journal rate limits prevent rapid failure loops. After hitting a
start limit, fix the sensor/network, then `sudo systemctl reset-failed
mid360s-driver.service mid360s-fastlio.service` and start both.
`network-online.target` does not guarantee radar connectivity: `enp3s0` needs
carrier and `192.168.1.50/24`; MID360S must be reachable at `192.168.1.194`.

Rollback: stop and disable both units. If previous units were replaced,
restore them from the printed backup, run `sudo systemctl daemon-reload` and
restore the recorded enable states. Do not launch a second chain via tmux.

### Acceptance of the latest deployed HEAD

Report three levels separately: static checks, ROS build, aircraft bench checks.
Passing static checks and compilation does not establish bench acceptance.

1. With propellers removed and no mission, perform one controlled `sudo reboot`.
   After reconnecting, check both `systemctl is-active`, `tmux ls` and
   `ros2 node list`. No `livox_ros_driver2_0` or `fast_lio_0` duplicate owner.
   Source Humble and the overlay in the checking shell.
2. Check `ros2 topic info /Odometry_highrate -v` and
   `ros2 topic info /LioHealth -v`: exactly one expected FAST-LIO publisher each.
   Use `timeout 10 ros2 topic hz TOPIC` for all five topics: lidar ~10 Hz,
   IMU ~200 Hz, lowrate odom at successful correction rate, highrate odom and
   health ~200 Hz.
3. Run `python3 python_sdk/testcode/lio_latency_probe.py --seconds 60`.
   This passive probe sends no FC commands and does not start Navigation.
   It compares source ROS epoch to callback system wall clock: real time,
   synchronized clocks and `use_sim_time=false` are required. Verify exact
   pairs, monotonic timestamps, rates, latency and unmatched counts. Shutdown
   boundary pending messages may cause a small mismatch; persistent mismatch
   needs investigation. Quantiles use up to the latest 120000 samples/topic;
   total counts, late counts, min/max and rewind counts cover the full run.
4. Without mount JSON, diagnostic services can run but `mis_boot`, calibration
   and Navigation enable must refuse. Never invent a reviewed mount.
5. Use controlled rejected-update replay/scene. An actual rejected iteration
   restores prediction, marks DEGRADED, leaves valid seq/anchor unchanged,
   skips lowrate odom/TF and map insertion, and continues housekeeping.
   A successful update failing the stricter quality gate can still publish
   lowrate odom and insert points; DEGRADED alone does not prove rejection.
6. With a measured reviewed mount and disarmed FC, verify 2 s stationarity,
   >=10 distinct valid corrections, fresh paired TRACKING data and calibration.
   Driver loss must invalidate pose and latch LOST; FAST-LIO restart changes
   epoch and requires ground reset, stationarity, valid corrections and a new
   basepoint. No flight actions are needed for these checks.
7. For >=5 min, run actual competition visual models, both cameras, server and
   mission-equivalent background threads **without mission flight actions**,
   alongside the probe with `--seconds 300`. Record P99, max, >50 ms and >100 ms
   counts. `STALE_SECONDS=0.05` stays fixed in this release. If overruns occur,
   investigate CPU contention, callback/executor delays, vision load, frequency
   and thermal throttling before changing it. P99 <40 ms with essentially no
   >50 ms events supports retaining the threshold.

`laser_corrected` gates lowrate odom/TF and map insertion. `correction_valid`
separately gates new valid seq/anchor and TRACKING authorization. Rejected
frames do not return early. Dynamic precision, sensor-loss control and parallel
vision latency cannot be inferred from these static checks.
