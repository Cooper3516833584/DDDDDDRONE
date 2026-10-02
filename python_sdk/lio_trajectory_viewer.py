"""MID360S/FAST-LIO passive 3D viewer, relative to the first TRACKING sample.

On the flight computer's graphical desktop/NoMachine terminal:
  source /opt/ros/humble/setup.bash
  source /home/fc/dddddrone/ros2_ws/install/setup.bash
  python3 /home/fc/dddddrone/python_sdk/lio_trajectory_viewer.py

Windows/local GUI preview without ROS: python lio_trajectory_viewer.py --demo
This diagnostic origin is independent of Navigation calibration and authorization.
"""

import argparse
from collections import OrderedDict, deque
import importlib.util
import math
from pathlib import Path
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


class TrajectoryModel:
    def __init__(self, mount_path=None, max_points=6000):
        provider = POSE.LioPoseProvider(mount_path)
        # Reuse the existing mount parser and transform; no production gate changes.
        self.mount_position, self.mount_rotation = provider._mount
        self.reference = "BODY" if provider.mount_reviewed else "LIDAR / UNREVIEWED MOUNT"
        if provider._reference_frame == "lidar":
            self.reference = "LIDAR ORIGIN"
        self.lock = threading.Lock()
        self.points = deque(maxlen=max_points)
        self.pending = (OrderedDict(), OrderedDict())
        self.origin = self.axes = self.epoch = None
        self.position = None
        self.last_stamp = 0
        self.received = 0.0
        self.status = "WAITING FOR ROS DATA"
        self.break_pending = False

    def reset_origin(self):
        with self.lock:
            self.origin = self.axes = self.position = None
            self.points.clear()
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
        if self.epoch is not None and self.epoch != epoch:
            self.origin = self.axes = self.position = None
            self.points.clear()
            self.last_stamp = 0
            self.pending[0].clear()
            self.pending[1].clear()
        self.epoch = epoch
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
            if self.received and time.monotonic() - self.received > POSE.LioPoseProvider.STALE_SECONDS:
                status = "STALE / NO FRESH PAIRED DATA"
                self.break_pending = True
            return list(self.points), self.position, status, self.reference


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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demo", action="store_true", help="Synthetic GUI preview; no ROS/hardware")
    parser.add_argument("--mount", type=Path, help="Optional existing mid360s_mount.json")
    parser.add_argument("--max-points", type=int, default=6000, help="Bounded trajectory history")
    parser.add_argument("--save-preview", type=Path, help="Save synthetic preview and exit (requires --demo)")
    args = parser.parse_args()
    if args.max_points < 2:
        parser.error("--max-points must be >=2")
    if args.save_preview and not args.demo:
        parser.error("--save-preview requires --demo")
    import matplotlib
    if args.save_preview:
        matplotlib.use("Agg")
    elif matplotlib.get_backend().lower() == "agg":
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    model = TrajectoryModel(args.mount, args.max_points)
    stop = threading.Event()
    worker = None
    if not args.demo:
        worker = threading.Thread(target=ros_worker, args=(model, stop), daemon=True)
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
            model.reset_origin()

        button.on_clicked(reset_display)

        def redraw(frame):
            if args.demo:
                t = time.monotonic() - started if not args.save_preview else 12.0
                points = [(i * 20000000, (0.08 * i / 50, 0.6 * math.sin(i / 50), 0.02 * i / 50))
                          for i in range(max(1, min(args.max_points, int(t * 50) + 1)))]
                position = points[-1][1]
                status, reference = "DEMO - SYNTHETIC DATA", "NO HARDWARE"
            else:
                points, position, status, reference = model.snapshot()
            xyz = [point[1] for point in points]
            if xyz:
                line.set_data_3d(*zip(*xyz))
            else:
                line.set_data_3d([], [], [])
            if position is not None:
                marker.set_data_3d([position[0]], [position[1]], [position[2]])
                coordinates.set_text("X: {:+.3f} m   Y: {:+.3f} m   Z: {:+.3f} m".format(*position))
            else:
                marker.set_data_3d([], [], [])
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
            plt.show()
    finally:
        stop.set()
        if worker is not None:
            worker.join(timeout=2)


if __name__ == "__main__":
    main()
