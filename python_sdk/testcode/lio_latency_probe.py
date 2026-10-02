"""Passive exact-stamp latency probe. No flight controller or Navigation imports."""

import argparse
from collections import OrderedDict, deque
import json
import math
import time


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


class LatencyStats:
    def __init__(self, max_samples=120000):
        self.samples = deque(maxlen=max_samples)
        self.count = self.late_50ms_count = self.late_100ms_count = 0
        self.timestamp_rewind_count = self.timestamp_duplicate_count = 0
        self.negative_latency_count = 0
        self.previous_stamp = None
        self.minimum = self.maximum = None

    def add(self, stamp_ns, receive_ns):
        latency = (receive_ns - stamp_ns) / 1e6
        if self.previous_stamp is not None:
            self.timestamp_rewind_count += stamp_ns < self.previous_stamp
            self.timestamp_duplicate_count += stamp_ns == self.previous_stamp
        self.previous_stamp = stamp_ns
        self.samples.append(latency)
        self.count += 1
        self.late_50ms_count += latency > 50
        self.late_100ms_count += latency > 100
        self.negative_latency_count += latency < 0
        self.minimum = latency if self.minimum is None else min(self.minimum, latency)
        self.maximum = latency if self.maximum is None else max(self.maximum, latency)

    def report(self):
        return {"count": self.count, "min_ms": self.minimum,
                "median_ms": percentile(self.samples, 0.5),
                "P95_ms": percentile(self.samples, 0.95),
                "P99_ms": percentile(self.samples, 0.99), "max_ms": self.maximum,
                "late_50ms_count": self.late_50ms_count,
                "late_100ms_count": self.late_100ms_count,
                "timestamp_rewind_count": self.timestamp_rewind_count,
                "timestamp_duplicate_count": self.timestamp_duplicate_count,
                "negative_latency_count": self.negative_latency_count,
                "quantile_sample_count": len(self.samples)}


class ExactStampPairs:
    def __init__(self, capacity=4096):
        self.pending = (OrderedDict(), OrderedDict())
        self.capacity = capacity
        self.paired_count = self.evicted_count = 0

    def add(self, side, stamp_ns):
        own, other = self.pending[side], self.pending[1 - side]
        if stamp_ns in other:
            del other[stamp_ns]
            self.paired_count += 1
        else:
            own[stamp_ns] = None
            if len(own) > self.capacity:
                own.popitem(last=False)
                self.evicted_count += 1

    def report(self):
        return {"paired_count": self.paired_count,
                "unpaired_count": self.evicted_count + sum(map(len, self.pending)),
                "evicted_unpaired_count": self.evicted_count,
                "boundary_pending_count": sum(map(len, self.pending))}


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    # Import ROS only when explicitly running the probe.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from fast_lio.msg import LioHealth

    rclpy.init()
    node = rclpy.create_node("lio_latency_probe")
    stats = (LatencyStats(), LatencyStats())
    pairs = ExactStampPairs()

    def receive(side, message):
        received = time.time_ns()
        stamp = message.header.stamp if side == 0 else message.state_stamp
        source = stamp_ns(stamp)
        stats[side].add(source, received)
        pairs.add(side, source)

    subscriptions = [
        node.create_subscription(Odometry, "/Odometry_highrate", lambda msg: receive(0, msg), qos_profile_sensor_data),
        node.create_subscription(LioHealth, "/LioHealth", lambda msg: receive(1, msg), qos_profile_sensor_data),
    ]
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, args.seconds - (time.monotonic() - started))))
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        report = {"seconds": elapsed, "Odometry_highrate": stats[0].report(),
                  "LioHealth": stats[1].report(), **pairs.report(),
                  "clock_basis": "source ROS epoch vs callback system wall clock; use_sim_time=false required"}
        for name in ("Odometry_highrate", "LioHealth"):
            report[name]["receive_rate_hz"] = report[name]["count"] / elapsed
        print(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if all(item.count for item in stats) and pairs.paired_count else 1


if __name__ == "__main__":
    raise SystemExit(main())
