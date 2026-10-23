# Autonomous Navigation Robot Skills

This document is the operating and engineering reference for building, testing,
and reviewing autonomous navigation behavior on Kenny. It is written for a
human operator and for an AI or engineer reviewing exported debugger packets.

## Mission

Kenny must do four things in order:

1. Establish where it is, with a known map frame and freshness/uncertainty.
2. Establish where it is allowed to go, using a validated destination and route.
3. Choose a motion command from measured inputs.
4. Prove that the command is safe to issue and distinguish the command from the
   motion the chassis actually produced.

When any step is uncertain, the correct action is to stop, preserve evidence,
and explain the blocking reason. A plausible guess is not a localization,
clearance measurement, arrival, or successful trip.

## Decision Contract

Every navigation decision should be reviewable as this chain:

```text
raw sensor packets
  -> timestamps, validity, calibration and coordinate frames
  -> localization belief and independent references
  -> destination and route
  -> policy observation features
  -> policy requested action
  -> deterministic safety guard
  -> permitted command
  -> motor transport command
  -> encoder/odom measured result
```

The debugger must show each boundary. Never label a requested command as an
executed command. Never label a motor command as physical movement until fresh
encoder or odometry evidence confirms it.

Each decision should carry:

- Monotonic decision ID and wall-clock timestamp.
- Map revision and map hash.
- Belief pose, pose source, age, and uncertainty.
- Independent pose evidence, such as surveyed ArUco or fresh odometry.
- Goal, marker ID, route revision, route length, and route blockage reason.
- Sensor feature groups with raw values, validity, and age.
- Model action before safety intervention.
- Guarded action, permission state, and exact guard reason.
- Requested, transported, and measured motion as separate values.
- Firmware state, watchdog age, encoder counts, and fault bits.

## Coordinate Frames

Use these conventions consistently:

- `map`: surveyed environment frame. Goals, markers, routes, and global pose
  estimates are expressed here.
- `odom`: locally continuous frame from wheel/IMU odometry. It is useful for
  short-term motion but can drift relative to `map`.
- `base_link`: Kenny chassis frame. Positive x is forward; positive yaw follows
  the ROS convention.
- Camera optical frames: use the calibrated camera transform. Do not compare a
  camera-relative point directly with a map point.
- LiDAR frame: apply the configured mount yaw and handedness before interpreting
  front, left, right, or rear clearance.

Every pose comparison must state the frame, timestamp, map revision, and
transform used. A disagreement without those four facts is not actionable.

## Localization Skills

### Surveyed ArUco

- Decode the configured dictionary and retain the marker ID and detection
  confidence.
- Verify the marker exists in the active map and that the map revision matches.
- Use measured marker size, camera intrinsics, depth support, marker orientation,
  and camera-to-base calibration.
- Require multiple fresh observations before using a marker to establish robot
  pose or arrival.
- Report rejected candidates, ambiguous IDs, depth gaps, size mismatch, and
  stale observations instead of silently dropping them.

### SLAM and Covariance

- Treat SLAM pose as a belief, not ground truth.
- Display covariance or uncertainty and its age.
- Do not use a cached pose as a current pose after a map change.
- When the active map changes, clear or invalidate pose associations until a
  fresh map-frame reference is established.

### Odometry

- Use encoder counts and IMU integration for local continuity and measured
  displacement.
- Track encoder sign, counts per revolution, wheel radius, wheel separation,
  invalid transitions, and time gaps.
- Separate encoder-derived pose from global map pose. Their difference is useful
  evidence, not automatically an error.
- A stale or rebooted motor bridge invalidates motion confirmation.

### Pose Reconciliation

For each comparison, calculate:

```text
position_error = hypot(x_a - x_b, y_a - y_b)
heading_error = atan2(sin(yaw_a - yaw_b), cos(yaw_a - yaw_b))
```

Record both errors, the source pair, timestamps, and thresholds. A large error
must block navigation or request operator review according to the configured
safety policy.

## Mapping Skills

- Keep the active map revision and SHA-256 hash with every decision.
- Treat map geometry, marker metadata, obstacle labels, and destination poses as
  safety-relevant configuration.
- Validate occupancy bounds, resolution, origin, marker uniqueness, marker
  dictionary, obstacle dimensions, and reachable destination points.
- Do not replace a map while a request is pending or running unless the system
  explicitly resolves the request and invalidates the old pose.
- After editing a map, compare the new and old marker coordinates, obstacle
  geometry, route reachability, and robot pose frame.
- Use the Map creator for geometry edits. Use debugger feedback to record why an
  edit is needed; do not turn free-text feedback into an automatic map change.
- After sync, latch STOP, re-establish localization, replan, and require fresh
  sensor readiness before motion.

## Route Planning Skills

- Plan in the current map revision and robot footprint, including inflation.
- Show whether the route came from the live refined map, saved prior map, or a
  commissioning fallback.
- Record route point count, planner result, goal pose, and the reason for an
  empty route.
- A direct commissioning route is not evidence that a normal autonomous route
  exists. It still requires live clearance and explicit commissioning mode.
- Replan when the map revision, goal, localization, or route validity changes.

## Sensor Safety Skills

For every sensor, show:

- Packet age and source timestamp age.
- Valid sample count and coverage.
- Calibration status and coordinate frame.
- Minimum measured distance and whether it includes floor returns.
- Whether values are measured, inferred, bridged, or unavailable.

Never replace missing data with a synthetic clear value. A missing validity bit
must remain missing through policy input and appear as a guard reason when it
blocks motion.

### LiDAR

- Check forward angular coverage, not only the nearest return.
- Account for the robot footprint and turning clearance.
- Reject stale scans and invalid ranges.
- Verify orientation with a known object before trusting front/side labels.

### RGB-D

- Separate RGB detection from depth-supported geometry.
- Check depth coverage, floor coverage, nearest obstacle, and measured minimum
  range.
- Treat missing depth as unknown, not free space.
- Block or reduce motion when the stopping envelope is not fully observed.

### IMU

- Report chip address, bus, calibration state, bias, and gaps.
- Do not use an uncalibrated or tilted IMU as an absolute heading source.
- Recalibrate after gaps, frame changes, or substantial tilt.

## Motion and Safety Skills

The safety guard is authoritative. It must run after policy inference and before
publishing a motion permission or motor command.

At minimum, check:

- Valid finite command and configured speed limits.
- Fresh LiDAR, depth, floor, and required downward inputs.
- Forward coverage and stopping distance.
- Turning footprint and side clearance.
- Floor drop or hazard evidence.
- Localization uncertainty and pose freshness.
- Motor bridge connection, firmware fault, watchdog, and E-stop.
- Command age and heartbeat health.

Use these states explicitly:

- `requested`: output selected by policy or a deterministic commissioning rule.
- `guarded`: output after clamping and safety checks.
- `permitted`: whether the guarded output may be sent.
- `transported`: what the motor bridge sent to firmware.
- `measured`: what encoders, odometry, and sensors show happened.

STOP must be fail-closed. Loss of dashboard heartbeat, controller heartbeat,
motor telemetry, or command freshness must remove motion permission. A software
STOP is not a substitute for a reachable physical power cutoff.

## Supervised Operation

Supervised operation is an evidence-gathering mode, not a way to bypass safety.

Before a supervised run:

1. Confirm the active map revision and starting pose.
2. Confirm the physical E-stop and motor-power cutoff are reachable.
3. Confirm all required sensors are fresh and coverage is sufficient.
4. Confirm wheels, surroundings, and intended path are visible to the operator.
5. Confirm the run duration, speed cap, clearance threshold, and automatic stop.
6. Latch STOP before changing modes or maps.

During a run, log the decision trace, operator observations, actual pose, actual
velocity, encoder counts, guard state, and any stop/restart event. At the end,
verify disarm and physical stopping before reviewing the result.

## Operator Feedback

Good feedback is specific, timestamped, and tied to a decision ID. Prefer:

```text
At decision 1842, ArUco ID 12 was visible at 0.42 m, but the map pose was
0.8 m away. The robot should have withheld motion and requested relocalization.
```

Include:

- What was observed.
- What the robot believed.
- What the operator expected instead.
- Which sensor or map fact supports the observation.
- Whether the issue is navigation, map, calibration, or safety.

Feedback is a hypothesis for review. It must not directly change a policy,
threshold, route, map, or motor command. Export the decision packet and
feedback journal, reproduce the behavior in simulation or replay, write a
regression test, then make a small reviewed code/configuration change.

## AI Review and Commit Workflow

An AI reviewing an exported packet should follow this order:

1. Check the decision ID, timestamps, map hash, and frame IDs.
2. Reconstruct requested, guarded, transported, and measured motion.
3. Identify the first invalid, stale, contradictory, or missing input.
4. Compare the guard reason with the actual evidence.
5. Classify the issue as sensor, calibration, localization, map, planner,
   policy, transport, UI, or documentation.
6. Propose the smallest safe change and list what it must not change.
7. Add or update a regression test using the observed packet.
8. Run tests, inspect the diff, and require human approval before commit.

An AI must not:

- Commit a change solely because a free-text observation sounds plausible.
- Remove a stop gate to make a test pass.
- Treat a model action as an explanation of why it is safe.
- Upload a map without revision and pending-job checks.
- Enable motors or issue a physical test command from feedback alone.
- Hide uncertainty, sensor gaps, or disagreement from the operator.

## Debugger Review Checklist

Before accepting a behavior change, answer yes to all applicable questions:

- Can the operator see the exact map revision and frame?
- Can the operator distinguish belief pose from independent/reference pose?
- Is every decision tied to fresh sensor ages and validity?
- Is the policy input visible or reproducible from the raw packet?
- Is the pre-guard command different from the guarded command when a guard
  intervened?
- Is the reason for withheld motion explicit?
- Is actual movement confirmed independently of the requested command?
- Can an operator record a comment without changing motion?
- Can the exported packet reproduce the decision offline?
- Is there a regression test for the observed failure?
- Does the change preserve fail-closed STOP behavior?

## Practical Test Ladder

Use the least hazardous test that can answer the question:

1. Static unit test with recorded packets.
2. Offline decision replay with no ROS or motors.
3. Live sensors with motor bridge disconnected.
4. Wheels raised, motor power disabled, encoder/serial checks only.
5. Wheels raised, bounded PWM test with physical stop reachable.
6. Clear floor, very low speed, short supervised run.
7. Longer supervised route with a human spotter.
8. Autonomous operation only after map, localization, stopping, and recovery
   evidence pass review.

Every step must have an explicit abort condition and produce a reviewable log.
