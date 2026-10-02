"""MID360S/FAST-LIO diagnostic 3D viewer, relative to the first TRACKING sample.

On the flight computer's graphical desktop/NoMachine, run this file directly
from VSCode. ROS2 Humble and the repository overlay are loaded automatically.

Windows/local GUI preview without ROS: python lio_trajectory_viewer.py --demo
Windows live mode starts a fresh local WSL map and stops it when the window closes.
This diagnostic origin is independent of Navigation calibration and authorization.
"""

import argparse
from collections import OrderedDict, deque
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def load_pose_code():
    # Load the pure positioning module without importing FlightController/__init__
    # (which configures log files and imports flight-controller communication).
    source = Path(__file__).resolve().parent / "FlightController/Components/LioPoseProvider.py"
    spec = importlib.util.spec_from_file_location("viewer_lio_pose_code", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


POSE = load_pose_code()


def prepare_ros_environment():
    """Use system ROS Python and setup scripts regardless of VSCode's shell."""
    if sys.platform != "linux" or os.environ.get("LIO_VIEWER_ROS_BOOTSTRAPPED") == "1":
        return
    setup = Path("/opt/ros/humble/setup.bash")
    overlay = Path(__file__).resolve().parents[1] / "ros2_ws/install/setup.bash"
    for required in (setup, overlay):
        if not required.is_file():
            raise RuntimeError(f"ROS environment missing: {required}; build ros2_ws first")
    result = subprocess.run(
        ["/bin/bash", "-c", 'source "$1" && source "$2" && env -0',
         "lio-viewer", str(setup), str(overlay)],
        check=True, capture_output=True, timeout=10,
    )
    environment = dict(os.environ)
    for entry in result.stdout.split(b"\0"):
        if b"=" in entry:
            key, value = entry.split(b"=", 1)
            environment[os.fsdecode(key)] = os.fsdecode(value)
    environment["LIO_VIEWER_ROS_BOOTSTRAPPED"] = "1"
    # Ubuntu ROS Humble bindings target system Python 3.10, not a VSCode venv.
    os.execve("/usr/bin/python3", ["/usr/bin/python3", str(Path(__file__).resolve()), *sys.argv[1:]], environment)


class TrajectoryModel:
    def __init__(self, mount_path=None, max_points=6000):
        provider = POSE.LioPoseProvider(mount_path)
        # Reuse the existing mount parser and transform; no production gate changes.
        self.mount_position, self.mount_rotation = provider._mount
        self.reference = "BODY" if provider.mount_reviewed else "LIDAR / UNREVIEWED MOUNT"
        if provider._reference_frame == "lidar":
            self.reference = "LIDAR ORIGIN"
        self.lock = threading.RLock()
        self.points = deque(maxlen=max_points)
        self.pending = (OrderedDict(), OrderedDict())
        self.origin = self.axes = self.epoch = None
        self.last_correction_seq = None
        self.position = None
        self.last_stamp = 0
        self.received = 0.0
        self.status = "WAITING FOR ROS DATA"
        self.break_pending = False
        self.generation = 0
        self.freshness_timeout = POSE.LioPoseProvider.STALE_SECONDS

    def reset_origin(self):
        with self.lock:
            self.origin = self.axes = self.position = None
            self.points.clear()
            self.generation += 1
            self.pending[0].clear()
            self.pending[1].clear()
            self.status = "WAITING FOR TRACKING ORIGIN"

    def receive(self, side, message):
        stamp = message.header.stamp if side == 0 else message.state_stamp
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        with self.lock:
            own, other = self.pending[side], self.pending[1 - side]
            own[stamp_ns] = (message, time.monotonic())
            if stamp_ns in other:
                odom, odom_received = self.pending[0].pop(stamp_ns)
                health, health_received = self.pending[1].pop(stamp_ns)
                self._pair(odom, health, stamp_ns, min(odom_received, health_received))
            while len(own) > 128:
                own.popitem(last=False)

    def _pair(self, odom, health, stamp_ns, received):
        epoch = int(health.epoch)
        seq = getattr(health, "correction_seq", None)
        if ((self.epoch is not None and self.epoch != epoch) or
                (seq is not None and self.last_correction_seq is not None and seq < self.last_correction_seq)):
            self.origin = self.axes = self.position = None
            self.points.clear()
            self.generation += 1
            self.last_stamp = 0
            self.pending[0].clear()
            self.pending[1].clear()
        self.epoch = epoch
        self.last_correction_seq = seq
        if stamp_ns <= self.last_stamp:
            self.status = "TIMESTAMP REWIND / DUPLICATE"
            self.break_pending = True
            return
        self.last_stamp = stamp_ns
        self.received = received
        age = time.time() - stamp_ns * 1e-9
        if age < -0.01 or age > POSE.LioPoseProvider.STALE_SECONDS:
            self.status = "STALE / CLOCK MISMATCH"
            self.break_pending = True
            return
        if int(health.state) != 1:
            self.status = {0: "INIT", 2: "DEGRADED", 3: "LOST"}.get(int(health.state), "UNKNOWN HEALTH")
            reason = getattr(health, "reason", "")
            if reason:
                self.status += " - " + reason
            self.break_pending = True
            return
        if odom.header.frame_id != "camera_init" or odom.child_frame_id != "imu":
            self.status = "UNEXPECTED ODOM FRAME"
            return
        pose = odom.pose.pose
        try:
            q = POSE._unit((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            offset = POSE._rotate(q, self.mount_position)
            current = tuple(float(value) + offset[i] for i, value in enumerate(
                (pose.position.x, pose.position.y, pose.position.z)))
            if not all(math.isfinite(value) for value in current):
                raise ValueError("Nonfinite position")
            if self.origin is None:
                gravity = health.gravity_o
                up = POSE._normalize((-gravity.x, -gravity.y, -gravity.z))
                forward = POSE._rotate(POSE._mul(q, self.mount_rotation), (1, 0, 0))
                projection = POSE._dot(forward, up)
                x_axis = POSE._normalize(tuple(forward[i] - projection * up[i] for i in range(3)))
                self.axes = (x_axis, POSE._normalize(POSE._cross(up, x_axis)), up)
                self.origin = current
            delta = tuple(current[i] - self.origin[i] for i in range(3))
            self.position = tuple(POSE._dot(axis, delta) for axis in self.axes)
        except (AttributeError, ValueError, TypeError):
            self.status = "INVALID POSE / GRAVITY"
            self.break_pending = True
            return
        if self.break_pending or (self.points and stamp_ns - self.points[-1][0] > 250000000):
            self.points.append((stamp_ns, (float("nan"),) * 3))
        self.break_pending = False
        # Plot at <=50 Hz while processing every received pair.
        if not self.points or stamp_ns - self.points[-1][0] >= 20000000:
            self.points.append((stamp_ns, self.position))
        self.status = "TRACKING (DISPLAY ONLY)"

    def snapshot(self):
        with self.lock:
            status = self.status
            if self.received and time.monotonic() - self.received > self.freshness_timeout:
                status = "STALE / NO FRESH PAIRED DATA"
                self.break_pending = True
            return list(self.points), self.position, status, self.reference

    def receive_packet(self, packet):
        with self.lock:
            if packet["generation"] != self.generation:
                self.points.clear()
                self.generation = packet["generation"]
            for stamp, position in packet["points"]:
                self.points.append((stamp, tuple(position) if position is not None else (float("nan"),) * 3))
            self.position = packet["position"]
            self.status = packet["status"]
            self.reference = packet["reference"]
            self.received = time.monotonic()


class LocalLocalizationSession:
    """Own only nodes started for this test; never stop existing ROS services."""
    def __init__(self):
        self.children = []
        self.session_lock = None

    @staticmethod
    def running(executable):
        found = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                if (entry / "exe").resolve().name == executable:
                    found.append(int(entry.name))
            except OSError:
                continue
        return found

    def launch(self, command):
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, errors="replace", start_new_session=True)
        tail = deque(maxlen=30)
        def drain():
            for index, line in enumerate(process.stdout):
                tail.append(line.rstrip())
                if index < 20:
                    print(f"[{Path(command[0]).name}] {line.rstrip()}", file=sys.stderr, flush=True)
        reader = threading.Thread(target=drain, daemon=True)
        self.children.append((process, tail, reader))
        reader.start()

    def start(self):
        import fcntl
        import hashlib
        from ament_index_python.packages import get_package_prefix
        repo = Path(__file__).resolve().parents[1]
        config = repo / "ros2_ws/src/FAST_LIO_ROS2/config/mid360s_drone.yaml"
        driver_config = repo / "ros2_ws/src/livox_ros_driver2/config/MID360s_config.json"
        for path in (config, driver_config):
            if not path.is_file():
                raise RuntimeError(f"Localization configuration missing: {path}")
        key = hashlib.sha256(str(repo).encode()).hexdigest()[:16]
        self.session_lock = open(f"/tmp/lio-viewer-{os.getuid()}-{key}.lock", "a")
        try:
            fcntl.flock(self.session_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another trajectory window is running; close it first")
        existing = self.running("fastlio_mapping")
        if existing:
            raise RuntimeError(f"FAST-LIO already running (PID {existing}); stop its terminal/service first")
        mapping = Path(get_package_prefix("fast_lio")) / "lib/fast_lio/fastlio_mapping"
        driver = Path(get_package_prefix("livox_ros_driver2")) / "lib/livox_ros_driver2/livox_ros_driver2_node"
        for executable in (mapping, driver):
            if not executable.is_file():
                raise RuntimeError(f"ROS executable missing: {executable}")
        if not self.running("livox_ros_driver2_node"):
            self.launch(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "run", "livox_ros_driver2",
                         "livox_ros_driver2_node", "--ros-args", "-p", "xfer_format:=1", "-p", "multi_topic:=0",
                         "-p", "data_src:=0", "-p", "publish_freq:=10.0", "-p", "output_data_type:=0",
                         "-p", "frame_id:=livox_frame", "-p", f"user_config_path:={driver_config}",
                         "-p", "cmdline_input_bd_code:=livox0000000001"])
        else:
            print("Using an existing Livox driver; its owner must stop it separately.", file=sys.stderr)
        self.launch(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "run", "fast_lio", "fastlio_mapping",
                     "--ros-args", "--params-file", str(config),
                     "-p", "use_sim_time:=false"])
        print("Started a fresh FAST-LIO map; closing this window stops its owned ROS nodes.", file=sys.stderr)

    def check(self):
        for process, tail, _ in self.children:
            if process.poll() is not None:
                raise RuntimeError(f"Localization node exited ({process.returncode}): " + " | ".join(tail))

    def close(self):
        for process, _, reader in reversed(self.children):
            if process.poll() is None:
                for sig, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                    try:
                        os.killpg(process.pid, sig)
                        process.wait(timeout=timeout)
                        break
                    except ProcessLookupError:
                        break
                    except subprocess.TimeoutExpired:
                        continue
            reader.join(timeout=1)
            process.stdout.close()
        self.children.clear()
        if self.session_lock is not None:
            self.session_lock.close()
            self.session_lock = None


def stream_local_ros(model, new_map=False):
    """Local WSL transport with optional owned ROS nodes; no flight commands."""
    stop = threading.Event()
    worker = threading.Thread(target=ros_worker, args=(model, stop), daemon=True)
    session = LocalLocalizationSession() if new_map else None
    previous_sigterm = None
    if sys.platform == "linux" and threading.current_thread() is threading.main_thread():
        previous_sigterm = signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())

    def commands():
        for line in sys.stdin:
            if line.strip() == "reset":
                model.reset_origin()
        stop.set()

    threading.Thread(target=commands, daemon=True).start()
    last_stamp, generation = -1, -1
    try:
        if session is not None:
            session.start()
        worker.start()
        while not stop.is_set():
            if session is not None:
                session.check()
            with model.lock:
                points, position, status, reference = model.snapshot()
                current_generation = model.generation
            if current_generation != generation:
                last_stamp, generation = -1, current_generation
            added = [(stamp, list(p) if all(math.isfinite(v) for v in p) else None)
                     for stamp, p in points if stamp > last_stamp]
            if added:
                last_stamp = added[-1][0]
            print(json.dumps({"type": "lio_trajectory", "generation": generation,
                              "points": added, "position": position,
                              "status": status, "reference": reference}), flush=True)
            stop.wait(0.05)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        stop.set()
        if worker.ident is not None:
            worker.join(timeout=2)
        if session is not None:
            session.close()
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)


class LocalWslConnection:
    def __init__(self, mount_path=None, max_points=6000, new_map=True):
        self.process = None
        self.lock = threading.Lock()
        self.mount_path = mount_path
        self.max_points = max_points
        self.new_map = new_map

    @staticmethod
    def translate_path(path):
        return subprocess.check_output(
            ["wsl.exe", "--exec", "wslpath", "-a", Path(path).resolve().as_posix()],
            text=True, timeout=10,
        ).strip()

    def close(self):
        with self.lock:
            if self.process is not None and not self.process.stdin.closed:
                try:
                    self.process.stdin.close()
                except OSError:
                    pass

    def reset(self, model):
        model.reset_origin()
        with self.lock:
            if self.process is not None and self.process.poll() is None and not self.process.stdin.closed:
                try:
                    self.process.stdin.write("reset\n")
                    self.process.stdin.flush()
                except OSError as exc:
                    with model.lock:
                        model.status = "LOCAL WSL ERROR: " + str(exc)

    def run(self, model, stop):
        try:
            translated = self.translate_path(__file__)
            command = ["wsl.exe", "--exec", "/usr/bin/python3", "-u", translated,
                       "--stream", "--max-points", str(self.max_points)]
            if self.new_map:
                command.append("--new-map")
            if self.mount_path is not None:
                command.extend(["--mount", self.translate_path(self.mount_path)])
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, encoding="utf-8", errors="replace", bufsize=1,
            )
            with self.lock:
                self.process = process
            error_tail = deque(maxlen=4)

            def errors():
                for line in process.stderr:
                    error_tail.append(line.rstrip())
                    print("[WSL]", line.rstrip())

            error_reader = threading.Thread(target=errors, daemon=True)
            error_reader.start()
            for line in process.stdout:
                if stop.is_set():
                    break
                try:
                    packet = json.loads(line)
                    if packet.get("type") == "lio_trajectory":
                        model.receive_packet(packet)
                except (ValueError, AttributeError, KeyError):
                    print("[WSL]", line.rstrip())
            if not stop.is_set():
                error_reader.join(timeout=0.5)
                raise RuntimeError("Local WSL ROS subscription exited: " + " | ".join(error_tail))
        except Exception as exc:
            with model.lock:
                model.status = "LOCAL WSL ERROR: " + str(exc)
                model.received = 0.0
        finally:
            self.close()  # EOF stops only our ROS subscriber.
            with self.lock:
                process = self.process
            if process is not None:
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    process.terminate()


def ros_worker(model, stop):
    node = context = executor = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from fast_lio.msg import LioHealth

        context = Context()
        rclpy.init(context=context)
        node = rclpy.create_node("lio_trajectory_viewer", context=context)
        node.create_subscription(Odometry, "/Odometry_highrate", lambda msg: model.receive(0, msg), qos_profile_sensor_data)
        node.create_subscription(LioHealth, "/LioHealth", lambda msg: model.receive(1, msg), qos_profile_sensor_data)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while not stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.05)
    except Exception as exc:
        with model.lock:
            model.status = "ROS ERROR: " + str(exc)
            model.received = 0.0
        print("ROS subscription failed:", exc)
        print("Run from a ROS2 Humble graphical terminal after sourcing the workspace.")
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


def set_line_xyz(artist, points):
    """Use arrays so Matplotlib can mask trajectory gaps and clipped points."""
    import numpy as np
    coordinates = np.asarray(points, dtype=float).reshape(-1, 3).T
    artist.set_data_3d(*coordinates)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demo", action="store_true", help="Synthetic GUI preview; no ROS/hardware")
    parser.add_argument("--mount", type=Path, help="Optional existing mid360s_mount.json")
    parser.add_argument("--max-points", type=int, default=6000, help="Bounded trajectory history")
    parser.add_argument("--save-preview", type=Path, help="Save synthetic preview and exit (requires --demo)")
    parser.add_argument("--stream", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--new-map", action="store_true", help="Own a fresh local ROS test session (Linux opt-in)")
    parser.add_argument("--subscribe-only", action="store_true", help="Windows: subscribe to existing ROS without restarting")
    args = parser.parse_args()
    if args.max_points < 2:
        parser.error("--max-points must be >=2")
    if args.save_preview and not args.demo:
        parser.error("--save-preview requires --demo")
    if not args.demo:
        prepare_ros_environment()
    if args.stream:
        stream_local_ros(TrajectoryModel(args.mount, args.max_points), args.new_map)
        return
    import matplotlib
    if args.save_preview:
        matplotlib.use("Agg")
    else:
        # VSCode/interactive settings may select an inline backend. This entry
        # point always needs a standalone window, including direct file runs.
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    model = TrajectoryModel(args.mount, args.max_points)
    stop = threading.Event()
    worker = None
    connection = LocalWslConnection(args.mount, args.max_points, not args.subscribe_only) if sys.platform == "win32" and not args.demo else None
    local_session = LocalLocalizationSession() if sys.platform == "linux" and args.new_map and not args.demo else None
    if local_session is not None:
        try:
            local_session.start()
        except BaseException:
            local_session.close()
            raise
    if not args.demo:
        if connection is not None:
            # Transport timeout only; WSL still applies the original 50 ms source gate.
            model.freshness_timeout = 0.25
        worker = threading.Thread(target=connection.run if connection else ros_worker,
                                  args=(model, stop), daemon=True)
        worker.start()
    try:
        fig = plt.figure(figsize=(11, 8))
        fig.canvas.manager.set_window_title("MID360S / FAST-LIO - 3D Trajectory")
        ax = fig.add_subplot(111, projection="3d")
        fig.subplots_adjust(top=0.80, bottom=0.12)
        ax.set_xlabel("X forward (m)")
        ax.set_ylabel("Y left (m)")
        ax.set_zlabel("Z up (m)")
        ax.set_box_aspect((1, 1, 1))
        ax.scatter([0], [0], [0], c="black", marker="+", s=100, label="Startup origin")
        line, = ax.plot([], [], [], color="#168aad", linewidth=1.7, label="Trajectory")
        marker, = ax.plot([], [], [], "o", color="#ef8354", markersize=7, label="Current / last pose")
        ax.legend(loc="upper left")
        coordinates = fig.text(0.07, 0.94, "X: --   Y: --   Z: --", fontsize=17, family="monospace")
        status_text = fig.text(0.07, 0.89, "Waiting for paired LIO data...", fontsize=11)
        fig.text(0.07, 0.84, "Fixed axes: startup heading forward / left / gravity up. Drag to rotate. Units: metres.", fontsize=9)
        button = Button(fig.add_axes([0.72, 0.025, 0.22, 0.045]), "Reset display origin")
        started = time.monotonic()

        def reset_display(event):
            nonlocal started
            started = time.monotonic()
            if connection is not None:
                connection.reset(model)
            else:
                model.reset_origin()

        button.on_clicked(reset_display)

        def redraw(frame):
            if local_session is not None:
                try:
                    local_session.check()
                except RuntimeError as exc:
                    with model.lock:
                        model.status = "LOCAL ROS ERROR: " + str(exc)
                        model.received = 0.0
                    stop.set()
            if args.demo:
                t = time.monotonic() - started if not args.save_preview else 12.0
                points = [(i * 20000000, (0.08 * i / 50, 0.6 * math.sin(i / 50), 0.02 * i / 50))
                          for i in range(max(1, min(args.max_points, int(t * 50) + 1)))]
                position = points[-1][1]
                status, reference = "DEMO - SYNTHETIC DATA", "NO HARDWARE"
            else:
                points, position, status, reference = model.snapshot()
            xyz = [point[1] for point in points]
            set_line_xyz(line, xyz)
            if position is not None:
                set_line_xyz(marker, [position])
                coordinates.set_text("X: {:+.3f} m   Y: {:+.3f} m   Z: {:+.3f} m".format(*position))
            else:
                set_line_xyz(marker, [])
                coordinates.set_text("X: --   Y: --   Z: --")
            status_text.set_text(status + " | " + reference)
            status_text.set_color("#168aad" if status.startswith(("TRACKING", "DEMO")) else "#bb3e03")
            finite = [(0, 0, 0)] + [p for p in xyz if all(math.isfinite(v) for v in p)]
            if position is not None:
                finite.append(position)
            lower = [min(p[i] for p in finite) for i in range(3)]
            upper = [max(p[i] for p in finite) for i in range(3)]
            span = max(1.0, *(upper[i] - lower[i] for i in range(3))) * 1.15
            for i, setter in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
                center = (lower[i] + upper[i]) / 2
                setter(center - span / 2, center + span / 2)

        redraw(0)
        if args.save_preview:
            fig.savefig(args.save_preview, dpi=120)
            plt.close(fig)
        else:
            animation = FuncAnimation(fig, redraw, interval=50, cache_frame_data=False)
            fig.canvas.mpl_connect("close_event", lambda event: stop.set())
            plt.show(block=True)
    finally:
        stop.set()
        if connection is not None:
            connection.close()
        if worker is not None:
            worker.join(timeout=22)
        if local_session is not None:
            local_session.close()


if __name__ == "__main__":
    main()
