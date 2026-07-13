#!/usr/bin/env python3
"""Probe remote hardware and display two Linux cameras through SSH on Windows.

The remote probe and capture programs are sent to ``python3`` over SSH stdin
and only live for the duration of this process.  They do not create files or
services on the remote computer.  The probe never opens a serial port.  Press
Q or Esc in either OpenCV window to stop.
"""

import argparse
import getpass
import os
import struct
import sys
import threading
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

try:
    import cv2
    import numpy as np
    import paramiko
except ImportError as exc:
    raise SystemExit(
        "Missing local dependency: {}. Install paramiko and opencv-python first.".format(
            exc.name
        )
    )


PACKET_MAGIC = b"DCAM"
PACKET_HEADER = struct.Struct("!4sBIII")
MAX_JPEG_SIZE = 20 * 1024 * 1024


REMOTE_CAPTURE_SOURCE = r'''
import struct
import sys
import time

import cv2


CAMERA_INDEXES = __CAMERA_INDEXES__
WIDTH = __WIDTH__
HEIGHT = __HEIGHT__
FPS = __FPS__
JPEG_QUALITY = __JPEG_QUALITY__
PACKET_MAGIC = b"DCAM"
PACKET_HEADER = struct.Struct("!4sBIII")


def open_camera(index):
    backends = []
    if hasattr(cv2, "CAP_V4L2"):
        backends.append(cv2.CAP_V4L2)
    backends.append(None)

    for backend in backends:
        if backend is None:
            capture = cv2.VideoCapture(index)
        else:
            capture = cv2.VideoCapture(index, backend)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            capture.set(cv2.CAP_PROP_FPS, FPS)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return capture
        capture.release()
    raise RuntimeError("Unable to open camera index {}".format(index))


def main():
    captures = []
    sequences = [0] * len(CAMERA_INDEXES)
    encode_options = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    frame_interval = 1.0 / FPS if FPS > 0 else 0.0
    output = sys.stdout.buffer

    try:
        for index in CAMERA_INDEXES:
            captures.append(open_camera(index))
            print(
                "camera {} opened".format(index),
                file=sys.stderr,
                flush=True,
            )

        while True:
            loop_started = time.monotonic()
            for slot, (index, capture) in enumerate(zip(CAMERA_INDEXES, captures)):
                ok, frame = capture.read()
                if not ok or frame is None:
                    print(
                        "camera {} frame read failed".format(index),
                        file=sys.stderr,
                        flush=True,
                    )
                    continue

                encoded_ok, encoded = cv2.imencode(".jpg", frame, encode_options)
                if not encoded_ok:
                    print(
                        "camera {} JPEG encode failed".format(index),
                        file=sys.stderr,
                        flush=True,
                    )
                    continue

                payload = encoded.tobytes()
                header = PACKET_HEADER.pack(
                    PACKET_MAGIC,
                    slot,
                    index,
                    sequences[slot],
                    len(payload),
                )
                output.write(header)
                output.write(payload)
                output.flush()
                sequences[slot] += 1

            remaining = frame_interval - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except (BrokenPipeError, ConnectionResetError, EOFError):
        return 0
    except Exception as exc:
        print("remote capture error: {!r}".format(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        for capture in captures:
            capture.release()
        print("remote cameras released", file=sys.stderr, flush=True)


raise SystemExit(main())
'''


REMOTE_HARDWARE_PROBE_SOURCE = r'''
import glob
import os
import subprocess
import sys


CAMERA_INDEXES = __CAMERA_INDEXES__
EXPECTED_SERIAL = (
    ("Flight controller", 0x66CC, 0x2233),
    ("HC-14 wireless serial", 0x1A86, 0x7523),
    ("CP2102 radar serial candidate", 0x10C4, 0xEA60),
)


def command_output(command):
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5,
            check=False,
        )
        return result.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return "unavailable: {}".format(exc)


def port_users(device):
    try:
        result = subprocess.run(
            ["fuser", device],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return "unknown"
    users = (result.stdout + " " + result.stderr).strip()
    return users if users else "none"


def serial_ports():
    try:
        from serial.tools.list_ports import comports
    except ImportError as exc:
        print("[ERROR] PySerial unavailable: {}".format(exc))
        return None
    return list(comports())


def probe_serial_devices():
    ports = serial_ports()
    if ports is None:
        return False

    print("\n=== Serial devices (enumeration only; ports are not opened) ===")
    if not ports:
        print("No serial devices found.")
    for port in sorted(ports, key=lambda item: item.device):
        vid = "{:04X}".format(port.vid) if port.vid is not None else "----"
        pid = "{:04X}".format(port.pid) if port.pid is not None else "----"
        print(
            "{} VID:PID={}:{} description={!r} users={}".format(
                port.device,
                vid,
                pid,
                port.description,
                port_users(port.device),
            )
        )

    all_found = True
    for label, expected_vid, expected_pid in EXPECTED_SERIAL:
        matches = [
            port
            for port in ports
            if port.vid == expected_vid and port.pid == expected_pid
        ]
        if matches:
            devices = ", ".join(port.device for port in matches)
            print(
                "[OK] {}: {} ({:04X}:{:04X})".format(
                    label, devices, expected_vid, expected_pid
                )
            )
        else:
            all_found = False
            print(
                "[MISSING] {} ({:04X}:{:04X})".format(
                    label, expected_vid, expected_pid
                )
            )
    return all_found


def open_camera(index, cv2):
    backends = []
    if hasattr(cv2, "CAP_V4L2"):
        backends.append(cv2.CAP_V4L2)
    backends.append(None)
    for backend in backends:
        capture = (
            cv2.VideoCapture(index)
            if backend is None
            else cv2.VideoCapture(index, backend)
        )
        if capture.isOpened():
            return capture
        capture.release()
    return None


def probe_cameras():
    print("\n=== Cameras (three-frame read test) ===")
    try:
        import cv2
    except ImportError as exc:
        print("[ERROR] remote OpenCV unavailable: {}".format(exc))
        return False

    all_ok = True
    for index in CAMERA_INDEXES:
        device = "/dev/video{}".format(index)
        users = port_users(device)
        if users not in ("none", "unknown"):
            all_ok = False
            print("[BUSY] camera index {} users={}".format(index, users))
            continue
        capture = open_camera(index, cv2)
        if capture is None:
            all_ok = False
            print("[MISSING] camera index {} cannot be opened".format(index))
            continue

        frames = 0
        shape = None
        try:
            for unused in range(3):
                ok, frame = capture.read()
                if ok and frame is not None:
                    frames += 1
                    shape = frame.shape
        finally:
            capture.release()

        if frames == 3:
            print("[OK] camera index {}: 3/3 frames, shape={}".format(index, shape))
        else:
            all_ok = False
            print("[FAILED] camera index {}: {}/3 frames".format(index, frames))
    return all_ok


def main():
    print("=== Remote host ===")
    print("hostname={}".format(command_output(["hostname"])))
    print("network:\n{}".format(command_output(["ip", "-brief", "address"])))
    print("\n=== USB devices ===")
    print(command_output(["lsusb"]))

    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    print("\n=== Stable serial paths ===")
    if by_id:
        for path in by_id:
            print("{} -> {}".format(path, os.path.realpath(path)))
    else:
        print("No /dev/serial/by-id entries found.")

    video_by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
    print("\n=== Stable video paths and indexes ===")
    if video_by_id:
        for path in video_by_id:
            print("{} -> {}".format(path, os.path.realpath(path)))
    else:
        print("No /dev/v4l/by-id entries found.")
    for sysfs_path in sorted(glob.glob("/sys/class/video4linux/video*")):
        node_name = os.path.basename(sysfs_path)
        name_path = os.path.join(sysfs_path, "name")
        try:
            with open(name_path, "r", encoding="utf-8") as name_file:
                product_name = name_file.read().strip()
        except OSError:
            product_name = "unknown"
        print("/dev/{} name={!r}".format(node_name, product_name))

    serial_ok = probe_serial_devices()
    camera_ok = probe_cameras()

    print("\n=== Flight-controller local service (informational) ===")
    listeners = command_output(["ss", "-lnt"])
    service_listening = any(
        field.endswith(":5654")
        for line in listeners.splitlines()
        for field in line.split()
    )
    print("FC server port 5654: {}".format(
        "LISTENING" if service_listening else "NOT LISTENING"
    ))
    print("\n[SAFE] No serial port was opened and no serial command was sent.")
    print("PROBE_RESULT serial_ok={} camera_ok={}".format(serial_ok, camera_ok))
    if not camera_ok:
        return 2
    return 0 if serial_ok else 1


raise SystemExit(main())
'''


class StreamState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest = {}  # type: Dict[int, Tuple[int, int, np.ndarray]]
        self.frame_times = {}  # type: Dict[int, Deque[float]]
        self.error = None  # type: Optional[BaseException]
        self.finished = threading.Event()


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display two cameras from the airborne computer over SSH."
    )
    parser.add_argument("--host", default="192.168.31.176")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--user", default="fc")
    parser.add_argument(
        "--cameras",
        type=int,
        nargs=2,
        metavar=("FIRST", "SECOND"),
        default=(2, 0),
        help="remote OpenCV camera indexes (default: front 2, downward 0)",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument(
        "--probe-only",
        action="store_true",
        help="run the read-only remote hardware probe without starting video",
    )
    parser.add_argument(
        "--skip-hardware-probe",
        action="store_true",
        help="skip USB, serial, occupancy, network, and camera preflight checks",
    )
    parser.add_argument(
        "--strict-host-key-checking",
        action="store_true",
        help="reject hosts not already present in the local SSH known-hosts file",
    )
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    if any(index < 0 for index in args.cameras):
        raise ValueError("camera indexes must be non-negative")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("width and height must be positive")
    if args.fps <= 0:
        raise ValueError("fps must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("jpeg-quality must be between 1 and 100")
    if args.connect_timeout <= 0 or args.startup_timeout <= 0:
        raise ValueError("timeouts must be positive")
    if args.probe_only and args.skip_hardware_probe:
        raise ValueError("--probe-only cannot be combined with --skip-hardware-probe")


def build_remote_source(args: argparse.Namespace) -> str:
    return (
        REMOTE_CAPTURE_SOURCE.replace("__CAMERA_INDEXES__", repr(tuple(args.cameras)))
        .replace("__WIDTH__", repr(args.width))
        .replace("__HEIGHT__", repr(args.height))
        .replace("__FPS__", repr(args.fps))
        .replace("__JPEG_QUALITY__", repr(args.jpeg_quality))
    )


def build_hardware_probe_source(args: argparse.Namespace) -> str:
    return REMOTE_HARDWARE_PROBE_SOURCE.replace(
        "__CAMERA_INDEXES__", repr(tuple(args.cameras))
    )


def run_hardware_probe(client, args: argparse.Namespace) -> int:
    timeout_seconds = max(5, int(args.startup_timeout))
    command = "timeout {}s python3 -u -".format(timeout_seconds)
    stdin, stdout, stderr = client.exec_command(command, get_pty=False)
    source = build_hardware_probe_source(args)
    stdin.write(source.encode("utf-8"))
    stdin.flush()
    stdin.channel.shutdown_write()

    output = stdout.read().decode("utf-8", "replace")
    error = stderr.read().decode("utf-8", "replace")
    exit_status = stdout.channel.recv_exit_status()
    if output:
        print(output.rstrip())
    if error.strip():
        print("[remote probe stderr] {}".format(error.strip()), file=sys.stderr)
    if exit_status == 124:
        print("Remote hardware probe timed out.", file=sys.stderr)
    return exit_status


def read_exact(stream, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            raise EOFError("SSH camera stream ended")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def receive_frames(stdout, state: StreamState) -> None:
    try:
        while True:
            header = read_exact(stdout, PACKET_HEADER.size)
            magic, slot, camera_index, sequence, payload_size = PACKET_HEADER.unpack(
                header
            )
            if magic != PACKET_MAGIC:
                raise RuntimeError("invalid remote camera packet header")
            if payload_size <= 0 or payload_size > MAX_JPEG_SIZE:
                raise RuntimeError(
                    "invalid JPEG payload size: {}".format(payload_size)
                )

            payload = read_exact(stdout, payload_size)
            encoded = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError(
                    "unable to decode frame from camera {}".format(camera_index)
                )

            now = time.monotonic()
            with state.lock:
                state.latest[slot] = (camera_index, sequence, frame)
                timestamps = state.frame_times.setdefault(slot, deque(maxlen=30))
                timestamps.append(now)
    except BaseException as exc:
        with state.lock:
            state.error = exc
    finally:
        state.finished.set()


def print_remote_stderr(stderr) -> None:
    for raw_line in iter(stderr.readline, b""):
        line = raw_line.decode("utf-8", "replace").rstrip()
        if line:
            print("[remote] {}".format(line), file=sys.stderr)


def calculate_fps(timestamps: Deque[float]) -> float:
    if len(timestamps) < 2:
        return 0.0
    elapsed = timestamps[-1] - timestamps[0]
    if elapsed <= 0:
        return 0.0
    return (len(timestamps) - 1) / elapsed


def run_viewer(args: argparse.Namespace) -> int:
    password = os.environ.get("FC_SSH_PASSWORD")
    if password is None:
        password = getpass.getpass(
            "SSH password for {}@{}: ".format(args.user, args.host)
        )

    client = paramiko.SSHClient()
    client.load_system_host_keys()
    if args.strict_host_key_checking:
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
    else:
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    channel = None
    state = StreamState()
    try:
        print("Connecting to {}@{}:{} ...".format(args.user, args.host, args.port))
        client.connect(
            args.host,
            port=args.port,
            username=args.user,
            password=password,
            timeout=args.connect_timeout,
            banner_timeout=args.connect_timeout,
            auth_timeout=args.connect_timeout,
            look_for_keys=False,
            allow_agent=False,
        )
        transport = client.get_transport()
        if transport is not None:
            transport.set_keepalive(10)

        if not args.skip_hardware_probe:
            print("Running read-only remote hardware probe ...")
            probe_status = run_hardware_probe(client, args)
            if args.probe_only:
                return 0 if probe_status == 0 else 1
            if probe_status not in (0, 1, 2):
                print(
                    "Hardware probe did not complete cleanly; refusing to start video.",
                    file=sys.stderr,
                )
                return 1
            if probe_status == 2:
                print(
                    "Camera preflight failed or a camera is already in use; "
                    "refusing to start the video stream.",
                    file=sys.stderr,
                )
                return 1
            if probe_status != 0:
                print(
                    "Serial hardware probe reported a missing device; video startup "
                    "will continue without opening any serial port.",
                    file=sys.stderr,
                )

        stdin, stdout, stderr = client.exec_command("python3 -u -", get_pty=False)
        channel = stdout.channel
        source = build_remote_source(args)
        stdin.write(source.encode("utf-8"))
        stdin.flush()
        stdin.channel.shutdown_write()

        receiver = threading.Thread(
            target=receive_frames, args=(stdout, state), daemon=True
        )
        stderr_reader = threading.Thread(
            target=print_remote_stderr, args=(stderr,), daemon=True
        )
        receiver.start()
        stderr_reader.start()

        print(
            "Waiting for remote cameras {}. Press Q or Esc to stop.".format(
                tuple(args.cameras)
            )
        )
        started = time.monotonic()
        received_any = False

        while True:
            with state.lock:
                frames = dict(state.latest)
                fps_values = {
                    slot: calculate_fps(timestamps)
                    for slot, timestamps in state.frame_times.items()
                }
                stream_error = state.error

            if frames:
                received_any = True
                for slot, (camera_index, sequence, frame) in sorted(frames.items()):
                    display = frame.copy()
                    label = "/dev/video{}  seq={}  {:.1f} FPS".format(
                        camera_index, sequence, fps_values.get(slot, 0.0)
                    )
                    cv2.putText(
                        display,
                        label,
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow("Remote camera {}".format(camera_index), display)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    return 0
            else:
                time.sleep(0.02)

            if not received_any and time.monotonic() - started > args.startup_timeout:
                raise TimeoutError("timed out waiting for the first remote camera frame")
            if state.finished.is_set():
                if isinstance(stream_error, EOFError) and received_any:
                    print("Remote camera stream closed.", file=sys.stderr)
                    return 1
                if stream_error is not None:
                    raise stream_error
                return 0
    finally:
        if channel is not None:
            channel.close()
        client.close()
        cv2.destroyAllWindows()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        validate_args(args)
        return run_viewer(args)
    except KeyboardInterrupt:
        print("Stopped by user.")
        return 130
    except Exception as exc:
        print("Error: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
