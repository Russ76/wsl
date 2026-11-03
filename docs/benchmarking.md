# Benchmarking

A benchmark run drives the robot along a version-controlled route, records
what the system produces, and compares it with Gazebo's ground truth.

```bash
ros2 launch wsl_benchmark benchmark.launch.py scenario:=warehouse_loop
```

```bash
ros2 run wsl_benchmark evaluate results/warehouse_loop_<date>_<time>
```

The first command writes `results/<scenario>_<date>_<time>/` and exits when the
route is finished. The second recomputes `metrics.json` from the raw files
without simulating again.

## Scenarios

`wsl_benchmark/config/scenarios/*.yaml` fixes everything that defines an
experiment: spawn pose, robot/sensor arguments and the route. A scenario file
is validated on load: the name must match the file, robot arguments must be
known, the route must start at the spawn pose, and the speed must be within
the MecanumDrive limits.

| Scenario | Route | Purpose |
|---|---|---|
| `warehouse_loop` | 116 m clockwise loop: staging area, east, south and west perimeter aisles, back to start | Flagship loop-closure experiment |
| `aisle_loop` | 62 m: cross aisle, aisle A2 between identical rack rows, west aisle | Repetitive geometry: rack bays repeat every 2.7 m |
| `warehouse_loop_lowres` | as `warehouse_loop`, LiDAR 8 × 450 instead of 16 × 900 | Sensitivity to sensor fidelity |
| `manual` | none | Record while driving with teleop until Ctrl-C |

The route scenarios use a +3% wheel-separation and −1% wheel-radius odometry
calibration error (see [architecture](architecture.md#decisions)), so wheel
odometry drifts the way an uncalibrated robot's does.

### Route driving

`route_follower` runs pure pursuit (1.0 m lookahead, in-place rotation when the
heading error exceeds 0.7 rad) on the **ground-truth** pose. It belongs to the
harness, not the autonomy stack: it reproduces the same physical path in every
run, so SLAM configurations are compared on identical input. It publishes only
`/cmd_vel`. It waits 3 s of sim time after the first ground-truth message and
for the `map` frame (60 s at most), then reports `completed`, `stuck` (< 0.5 m
of progress in 30 s) or `timeout`.

## Recorded data

The recorder only subscribes (plus TF lookups). It writes raw observations, and
`evaluate.py` derives every number from them.

| File | Content |
|---|---|
| `run.json` | scenario, route outcome, expected rates, first/last `/clock` with wall time, CPU count |
| `sensor_stamps.csv` | per message: topic, header stamp, wall arrival time |
| `ground_truth.csv` | `/ground_truth/odom` (world frame), 50 Hz |
| `odometry.csv` | `/odom` (odom frame), 50 Hz |
| `slam_online.csv` | per `/odom` sample: latest `map→odom` ∘ odometry pose, i.e. the SLAM pose available online |
| `slam_optimized.csv` | final RTAB-Map graph poses (from `/rtabmap/mapData`) with each node's capture stamp |
| `perception.csv` | per cloud: raw points, filtered points, end-to-end latency |
| `clusters.csv` | per cloud: cluster count |
| `perception_diagnostics.csv` | per diagnostics window: clouds, rejected, mean/max processing time |
| `slam_info.csv` | per RTAB-Map update: loop-closure id, proximity-detection id, `Timing/Total/ms` |
| `resources.csv` | 1 Hz per process: component, CPU % of one core, RSS |

Sensor streams are subscribed **reliable**, so the statistics describe what
the producer emitted, not DDS losses of a best-effort subscriber. Camera
images are read raw: only the CDR header stamp is decoded, so the recorder
does not deserialize 18 MB/s of pixels in Python.

## Metrics

### Sensor timing

For the header stamps `t_0..t_{n-1}` of one stream, in arrival order, with
`Δ_i = t_{i+1} − t_i`:

- rate = `(n − 1) / (max t − min t)`, in **simulated** time
- period mean / min / max / std over the positive `Δ_i`
- regressions = `#{Δ_i < 0}`, duplicates = `#{Δ_i = 0}`
- stalls = `#{Δ_i > 2 / f_expected}`
- rate ratio = rate / `f_expected`

### Perception

- reduction = `1 − Σ filtered / Σ raw` over all clouds
- in-node processing = the node's own steady-clock time per cloud, from
  `/diagnostics` (cloud-weighted mean of the 1 s window means, max of maxima)
- end-to-end latency = wall arrival of `/perception/filtered_points` −
  wall arrival of the `/lidar/points` message with the same stamp. This
  includes DDS transport to the perception node, processing, serialization and
  transport to the recorder. It excludes bridge time, which happens before
  both arrivals. Outputs the recorder handles before their input (callback
  ordering in its executor) have no latency; they are reported as
  `unmatched_outputs` (0 of ~2000 per run in the results below). The algorithm cost without
  transport or CPU contention is measured separately by `pipeline_benchmark`.

### Trajectory accuracy

All trajectories are compared in SE(2) (x, y, yaw). The robot is planar and
RTAB-Map runs with `Reg/Force3DoF`.

**Association.** Each estimate sample at time `t` is paired with ground truth
linearly interpolated at `t` (yaw along the shortest arc). A sample is dropped
if `t` is outside the ground-truth span or the bracketing ground-truth samples
are more than 0.1 s apart, so dropouts are never bridged. Ground truth is
50 Hz, so the interpolation step is ≤ 20 ms.

**ATE** (absolute trajectory error, Sturm et al. 2012): find the rigid
transform `(R, t)` minimising `Σ ‖g_i − (R p_i + t)‖²` over the associated
positions, using Umeyama's closed form without scale and with the reflection
guard (`det R = +1`). Then report RMSE, mean, median, max and std of
`‖g_i − (R p_i + t)‖`, plus the heading error after the same rotation. ATE
measures global shape consistency and needs no shared origin.

**Drift** (online view): align only the **first** pose exactly, then report the
final and maximum position error, the final error as a percentage of the
ground-truth path length, and the final heading error. This is what an
operator would see if the robot started from a known pose.

**RPE** (relative pose error, distance-based as in KITTI): for each `i`, `j` is
the first sample whose ground-truth path length from `i` is ≥ Δ (default 1 m,
1 nm tolerance). The error is `E = (Q_i⁻¹ Q_j)⁻¹ (P_i⁻¹ P_j)` with `Q` the ground
truth and `P` the estimate, as SE(2) matrices. Report the translation and
rotation statistics of `E`. RPE is independent of global alignment and
measures local consistency (e.g. 2% scale error ⇒ 0.02 m per 1 m).

Three estimates are evaluated:

| Estimate | Meaning |
|---|---|
| `odometry` | wheel odometry alone: the baseline SLAM must beat |
| `slam_online` | the pose a consumer had at each instant: errors before a loop closure stay in this curve |
| `slam_optimized` | the final graph after all closures: the map's consistency |

All formulas are unit-tested against analytic cases in
`wsl_benchmark/test/test_metrics.py`: rigid copies give zero ATE and RPE,
symmetric lateral noise ±d gives RMSE d, a heading error θ at the start gives
final drift `2L sin(θ/2)`, and a 2% scale error gives RPE 0.02 m per metre.

### SLAM

- proximity detections: RTAB-Map loop closures found by scan proximity. This is
  the closure mechanism in this LiDAR-only configuration.
- loop closures: RTAB-Map's appearance-based (bag-of-words) closures. Always 0
  here, because there is no camera input to RTAB-Map.
- processing time: `Timing/Total/ms` from `/rtabmap/info`, per update (1 Hz).

### System

- real-time factor = Δ sim time / Δ wall time between the first and last `/clock`
- per component (Gazebo, SLAM, perception, bridge, robot_state_publisher,
  recorder): CPU % of one core (psutil, 1 Hz), max RSS, and RSS growth over
  the run after a 20 s warm-up

## Results

One run per scenario, all routes completed, on the host described under
limitations.

| | `warehouse_loop` | `aisle_loop` | `warehouse_loop_lowres` |
|---|---|---|---|
| Path driven | 114.8 m | 61.3 m | 114.8 m |
| Duration (sim) | 199.6 s | 131.5 s | 199.7 s |
| Real-time factor | 0.929 | 0.923 | 0.931 |
| Odometry ATE RMSE | 1.252 m | 0.632 m | 1.253 m |
| Odometry final drift | 4.176 m (3.64%), 13.97° | 1.869 m (3.05%), 13.97° | 4.176 m (3.64%), 13.97° |
| SLAM online ATE RMSE | 0.420 m | 0.256 m | 1.021 m |
| SLAM optimized ATE RMSE | 0.320 m | 0.222 m | 1.028 m |
| SLAM optimized RPE per metre | 0.030 m, 0.056° | 0.029 m, 0.076° | 0.080 m, 0.132° |
| Proximity detections (first at) | 6 (191.4 s) | 7 (122.4 s) | 5 (192.4 s) |
| SLAM processing, mean / p95 | 125 / 337 ms | 168 / 462 ms | 64 / 146 ms |
| Perception in-node, mean / max | 11.2 / 22.3 ms | 11.6 / 32.8 ms | 3.2 / 10.2 ms |
| Perception end-to-end, mean / p95 | 11.6 / 16.0 ms | 12.0 / 16.9 ms | 3.3 / 4.2 ms |
| Perception reduction | 54.9% | 50.7% | 34.9% |
| Sensor stalls / regressions | 0 / 0 | 1 / 0 | 0 / 0 |

The odometry heading error is the calibration model: 360° × (1 − 0.99 / 1.03) =
13.98° over a closed loop, against 13.97° measured.

## Limitations

- **Simulated sensors.** Gazebo noise models are Gaussian and stationary.
  There is no motion distortion of LiDAR sweeps, no multipath, and no rolling
  shutter. Results transfer to hardware only qualitatively.
- **Odometry error is systematic by design.** The calibration-error model makes
  drift repeatable. With calibration scales of 1.0 the simulated wheels do not
  slip at 1 m/s²: odometry error was 0.001 m and 0.00° over the drive test.
  Real rollers slip and have play, which this friction model does not include.
- **Single run per scenario.** Gazebo, rendering and DDS timing are not
  bit-deterministic, so repeated runs differ slightly. Report several runs
  before drawing fine-grained conclusions.
- **Ground-truth-driven route.** The path is reproducible, but it does not
  include navigation errors a real autonomy stack would add.
- **Wall-clock metrics depend on the host.** Latency, CPU and RTF were
  measured in an 8-CPU arm64 container on an Apple M1 Pro with **software
  rendering** (Mesa llvmpipe).
  A GPU changes Gazebo's share dramatically.
- **Recorder memory grows with run length.** It keeps all rows in memory until
  the end (about 98 MB RSS, +11 MB over a 3.6-minute run). This is expected,
  not a leak, and appears in `resources`.
- **Observer cost.** The recorder is a Python process subscribing to every
  sensor; its CPU is reported so it can be subtracted, but it competes for the
  same cores.
