# N97 navigation CPU3 check — 2026-10-04

## Implementation and deployment

The deployed N97 has four physical cores and four logical CPUs, numbered 0–3.
Ubuntu is 22.04.5, kernel 6.8.0-138-generic, Python 3.10.12, systemd 249.
The FC server, MID360S driver and FAST-LIO services have no CPU affinity or
CPU quota configured. Other processes can still use CPU3; this change does
not reserve an exclusive core.

`Navigation.start()` starts its named `navigation_cpu3` thread, waits up to
2 seconds for affinity setup, then starts height and watchdog threads.
On Linux the navigation thread sets and verifies affinity `{3}` before
entering `_navigation_task()`. Failure stops startup, sends the existing
zero control and raises an exception. Each restart binds the new thread.
Non-Linux offline usage retains its existing behavior.

The lower-layer change is limited to
`python_sdk/FlightController/Solutions/Navigation.py`. Public signatures,
flight protocol, PID gains, units, source freshness thresholds and navigation
loop frequency are unchanged. Commits `823ae59` and `8fb7bf8` were pushed to
main and deployed through `git pull --ff-only` from clean device checkouts.

## Verification

- Local targeted checks: 54 tests and 5 subtests passed, including affinity
  success, restart, permission failure, unavailable CPU, incorrect readback,
  navigation stale pose behavior, waypoint sequencing and LIO provider gates.
- Four affinity tests also passed on the N97.
- `testcode/navigation_cpu3_probe.py` uses actual Navigation and real ROS
  subscriptions, with a simulated FC state and an in-memory output sink.
  It never connects to FC, serial devices, or services, and suppresses debug
  disk logging during measurement. Diagnostic calibration waits up to 25 s
  without relaxing any production gate.
- Pinned 10 s round: 448/448 affinity checks and scheduler samples were CPU3.
  Height/watchdog threads retained CPU0–3. Odometry, health and IMU diagnostic
  callbacks were about 199.7 Hz, with maximum gaps about 38.7–39.8 ms and no
  nonmonotonic timestamps. Stop, stale-pose horizontal zero, stop-event zero
  and final all-zero output checks passed.
- Overall pinned test **failed**: only 3/448 sampled poses remained valid;
  navigation automatically disabled. The second start bound CPU3 successfully
  but failed to obtain the stationary IMU calibration window within 25 s.
- Same diagnostic with an unpinned test thread also **failed**: 5/445 valid
  pose samples; callbacks about 197.7 Hz and maximum gaps about 51.1–51.2 ms.
  Thus the observed pose loss is not exclusive to CPU3 binding. This comparison
  modified only a temporary diagnostic object, not deployed production files.
- Independent passive IMU sampling received 2301 messages in 12 s, with no
  source gap above 20 ms. Eleven gyro samples exceeded the existing 0.02 rad/s
  stationary gate; maximum was 0.02107 rad/s. The production gate remains intact.
- One diagnostic shutdown reported an rclpy Destroyable exception. All diagnostic
  navigation threads stopped; this shutdown warning has not been resolved.

## Result and limits

CPU3 binding and startup failure handling work. Complete navigation readiness
has **not** passed the live test. The existing busy navigation loop and Python
GIL may contribute to callback delay, but the cause of provider invalidation
is not established by these measurements. Do not equate topic frequency with
usable Navigation pose. Real FC transmission, motor-on closed-loop flight,
vision contention, production debug logging and long-duration behavior were
not tested. No production service was restarted and no flight command was sent.

To revert the affinity behavior, revert the source change locally, validate and
push it, then fast-forward the device checkout. Do not edit device repository
files directly. A newly launched mission uses the deployed CPU3 behavior.
