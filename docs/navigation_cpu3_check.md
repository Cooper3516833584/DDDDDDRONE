# N97 navigation CPU3 check — 2026-10-04

**Current result:** the follow-up fixes were approved, deployed and passed two
20 s live simulated closed-loop rounds with production debug logging enabled.
The sections before "Follow-up diagnosis and repair" below preserve the initial
failed test and its then-current implementation; their limitations are historical.

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

## Follow-up diagnosis and repair

Instrumentation caught the navigation thread latching the provider lost state
when the paired pose source age reached 50.035 ms, exceeding the unchanged
50 ms freshness gate. Health was still TRACKING and receive age was about
33 ms. Once latched, the provider correctly clears its base point and requires
ground recovery. Average topic frequency alone did not expose this transient.
The same failure occurred without affinity. Yielding the busy navigation loop
for 5 ms allowed the ROS callbacks to update the paired pose consistently;
the evidence implicates Python GIL/provider lock contention in the busy loop.

The approved protected lower-layer repair is in
`python_sdk/FlightController/Solutions/Navigation.py`:

- Sleep 5 ms on each navigation iteration and recheck the running flag.
- Initialize the PID pause state as paused and mark it paused on invalid pose,
  so initial enable and explicit re-enable restore PID automatic mode.

This fixes a second bug: an initial missing pose disabled the PIDs, but the
previous pause state could prevent re-enabling them after calibration. A valid
pose and an enabled navigation flag therefore did not guarantee control output.
The diagnostic now requires enabled PIDs and continuing nonzero horizontal
output, allowing up to 0.2 s for asynchronous startup before measurement.
It wraps the existing provider callbacks without adding duplicate subscribers
and drains the diagnostic executor before shutdown; the final run had no
Destroyable exception.

CPU3 affinity, PID gains, protocols, units, validity/health checks, unlock and
mode checks, IMU calibration gates and explicit re-enable requirements remain
intact. Production calibration timeout remains 3 s; only the diagnostic waits
up to 25 s. User approval covered deployment of the 5 ms yield and PID repair.
Code commits `7840264` and `8411e23` were pushed and deployed by fast-forward pull.

### Final verification (17:23–17:24, Asia/Shanghai)

Actual Navigation and live lidar/IMU/FAST-LIO streams were used on the N97.
FC state and output remained simulated in memory. Production debug logging
was enabled. Each round included a stop and restart, ground calibration,
20 s steady-state measurement, deliberate stale-pose injection and stop checks.

| Metric | Round 1 | Round 2 |
| --- | ---: | ---: |
| Duration | 20.001 s | 20.008 s |
| Valid pose / samples | 891 / 891 | 889 / 889 |
| CPU3 affinity and scheduled CPU / samples | 891 / 891 | 889 / 889 |
| Navigation and all PIDs enabled / samples | 891 / 891 | 889 / 889 |
| PID startup | 8.61 ms | 5.35 ms |
| Nonzero horizontal output frames | 2836 | 2808 |
| Output rate | 141.8 Hz | 140.3 Hz |
| Maximum nonzero output gap | 19.55 ms | 19.05 ms |
| IMU / odometry / health callback rates | 199.99 Hz | 199.97 Hz |
| Maximum callback source age | 22.13 ms | 16.67 ms |
| Raw gyro samples above 0.02 rad/s | 1 | 0 |

All 1780 samples passed. Both rounds passed stale-pose disable/horizontal zero,
stop-event zero, complete thread shutdown and final all-zero output checks.
Source timestamps were monotonic. Height and watchdog threads retained CPU0–3.
The probe returned `passed: true` with exit code 0. Local targeted validation
passed 57 tests plus 5 affinity subtests; the four affinity tests also previously
passed on the N97.

### Separate IMU stationary-gate finding

A passive 20 s sample contained 4001 IMU messages, no source gap above 20 ms,
and two raw gyro samples above 0.02 rad/s (maximum 0.020019 rad/s). The longest
continuous below-threshold interval was 15.39 s. Mean raw angular velocity was
approximately [-0.00625, 0.00558, 0.01266] rad/s; FAST-LIO corrected means were
approximately [0.000027, 0.000052, -0.000016] rad/s. Yaw changed by -0.0048 degrees
over the sample, with a 0.263 degree range.

These observations suggest raw gyro bias plus noise occasionally exceeds the
conservative per-frame stationary threshold. They do not indicate transport
gaps or a unit conversion error. This gate affects ground calibration; it is
not part of the post-calibration freshness check that caused the pose loss.
The threshold and continuous 2 s stationary requirement were retained. Both
final diagnostic calibrations passed, but occasional raw outliers and a
production 3 s calibration refusal remain possible. No bias compensation or
threshold relaxation was introduced.

### Current limits and changed files

The live simulated test now passes with production logging. Real FC
transmission, motor-on flight, vision contention and long-duration operation
remain untested. No production service was restarted and no flight command
was sent. An already-running mission does not reload this source change.

Follow-up files: the protected `Navigation.py`,
`python_sdk/testcode/navigation_cpu3_probe.py`,
`python_sdk/testcode/test_navigation_lio_integration.py`, and this record.
