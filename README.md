<div align="center">

# Warehouse Simulation Lab

**A warehouse mobile robot that maps, perceives, and measures its own accuracy.**

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](#supported-platforms)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)](#supported-platforms)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?logo=ubuntu&logoColor=white)](#ubuntu-2404)
[![macOS](https://img.shields.io/badge/macOS-Intel_%7C_Apple_Silicon-000000?logo=apple&logoColor=white)](#macos)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#perception-pipeline)
[![Python 3](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)](#benchmark)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue)](LICENSE)
![Version](https://img.shields.io/badge/version-0.1.0-brightgreen)

[Quick start](#quick-start) ·
[Demo](#demo) ·
[Performance](#performance) ·
[How it works](#how-it-works) ·
[Testing](#testing) ·
[Docs](#documentation)

</div>

<br/>

<div align="center">
  <img src="docs/media/dashboard.gif" alt="Gazebo, RViz, robot camera and SLAM floor plan running side by side" width="92%"/>
  <br/>
  <sub>One run, four views: Gazebo follow cam · RViz SLAM · onboard camera · floor plan with the SLAM path</sub>
</div>

## Overview

WSL is a ROS 2 Jazzy and Gazebo Harmonic platform for developing and measuring
perception and mapping on a warehouse mobile robot. It answers engineering
questions with numbers: how far wheel odometry drifts, how much loop closure
recovers, what LiDAR resolution costs, and where the CPU goes.

| Component | Description |
|---|---|
| **Robot** | Robotnik RB-Summit-XL Steel with mecanum wheels, 3D LiDAR, RGB camera and IMU |
| **World** | 40 × 30 m warehouse with repetitive racks, pillars and dead ends |
| **Perception** | C++ / PCL point-cloud pipeline: filtering, ground removal, obstacle clusters |
| **SLAM** | RTAB-Map 3D LiDAR graph SLAM with ICP and loop closure |
| **Benchmarks** | Every estimate scored against ground truth: ATE, RPE, drift, latency, CPU |

## Demo

<table>
  <tr>
    <td width="50%" align="center">
      <img src="docs/media/gazebo-follow.gif" alt="Gazebo follow camera on the Summit Steel between racks" width="100%"/><br/>
      <b>Gazebo</b><br/><sub>The Summit Steel driving between racks</sub>
    </td>
    <td width="50%" align="center">
      <img src="docs/media/rviz-slam.gif" alt="RViz with robot model, LiDAR rings, SLAM map and clusters" width="100%"/><br/>
      <b>RViz + RTAB-Map</b><br/><sub>Robot model, LiDAR, SLAM map and obstacle clusters</sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/media/robot-camera.gif" alt="Robot onboard camera stream" width="100%"/><br/>
      <b>Onboard camera</b><br/><sub><code>/camera/image_raw</code> at 640 × 480, 20 Hz</sub>
    </td>
    <td width="50%" align="center">
      <img src="docs/media/rviz-model.gif" alt="RViz close-up of the robot model with TF axes and turning wheels" width="100%"/><br/>
      <b>Robot model</b><br/><sub>TF axes, wheels turning from <code>/joint_states</code></sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/media/robot-closeup.png" alt="Summit Steel close-up in Gazebo" width="100%"/><br/>
      <b>Close-up</b><br/><sub>Robotnik meshes in Gazebo</sub>
    </td>
    <td width="50%" align="center">
      <img src="docs/media/robot-aisle.png" alt="Summit Steel in a warehouse aisle" width="100%"/><br/>
      <b>In an aisle</b><br/><sub>Between rack rows in aisle A2</sub>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/media/warehouse-overview.png" alt="Top-down view of the warehouse" width="100%"/><br/>
      <b>Warehouse</b><br/><sub>40 × 30 m, top down</sub>
    </td>
    <td width="50%" align="center">
      <img src="docs/media/tf-tree.png" alt="TF tree from tf2_tools view_frames" width="100%"/><br/>
      <b>TF tree</b><br/><sub><code>tf2_tools view_frames</code>, with SLAM running</sub>
    </td>
  </tr>
</table>

## Quick start

### Ubuntu 24.04

Requires Ubuntu 24.04 LTS with ROS 2 Jazzy installed at `/opt/ros/jazzy`.

**1. Get the code**

```bash
git clone https://github.com/azerbenali/wsl.git
cd wsl
```

**2. Install dependencies**

```bash
source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

**3. Build**

```bash
colcon build --symlink-install
source install/setup.bash
```

**4. Launch**

```bash
ros2 launch wsl_sim warehouse.launch.py
```

**5. Drive** (second terminal)

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### macOS

On macOS (Intel or Apple Silicon), WSL runs in a ready-made Ubuntu 24.04 / ROS 2
Jazzy environment as a VS Code dev container. Gazebo and RViz open in your browser.

| Step | Action |
|:---:|---|
| 1 | Install **Docker Desktop** and **VS Code** with the **Dev Containers** extension |
| 2 | In Docker Desktop → *Settings → Resources*, allocate at least **4 CPUs** and **8 GB** memory |
| 3 | Open the `wsl` folder in VS Code and choose **Reopen in Container** (first start installs ROS, Gazebo, RTAB-Map and builds the workspace, about 10–20 minutes) |
| 4 | In the VS Code terminal: `ros2 launch wsl_sim warehouse.launch.py` |
| 5 | Open <http://localhost:6080/vnc.html?autoconnect=1&resize=scale> to see Gazebo and RViz |

Build, test, run, benchmark and debug are available as VS Code tasks
(**Terminal → Run Task**) and debug configurations (**Run and Debug**).
The same dev container works on Windows.

## Performance

<details>
<summary><b>System metrics</b></summary>

<br/>

| System (`warehouse_loop`) | Measured |
|---|---|
| LiDAR / camera / IMU / odom rate (sim time) | 10.000 / 20.000 / 100.001 / 50.000 Hz, 0 stalls, 0 timestamp regressions |
| Perception processing inside the node | 11.2 ms mean, 22.3 ms max (period 100 ms), 0 rejected clouds |
| Perception end-to-end latency | 11.6 ms mean, 16.0 ms p95 |
| Perception point reduction (14 400 raw → ~6 490 filtered) | 54.9 % |
| Perception algorithm cost in isolation (`pipeline_benchmark`) | 7.1 ms mean, 7.3 ms p95 |
| Proximity loop closures | 6, first one when the loop closes (t = 191.4 s) |
| Real-time factor, full stack | 0.93 |
| CPU (% of one core): Gazebo / SLAM / perception / bridge / recorder | 268 / 23 / 15 / 11 / 43 |

Measured in an 8-CPU arm64 container on an Apple M1 Pro with Mesa llvmpipe.

Methodology: [docs/benchmarking.md](docs/benchmarking.md) ·
Profiling: [docs/architecture.md](docs/architecture.md#performance)

</details>

## How it works

```mermaid
flowchart LR
    GZ["Gazebo Harmonic<br/>warehouse + Summit Steel"] -->|ros_gz_bridge| L["/lidar/points"]
    GZ --> C["/camera/image_raw"]
    GZ --> I["/imu/data"]
    GZ --> O["/odom + TF"]
    L --> P["wsl_perception<br/>filter · ground · clusters"]
    L --> S["RTAB-Map<br/>ICP graph SLAM"]
    O --> S
    P --> R["RViz"]
    S --> R
    S --> B["wsl_benchmark"]
    P --> B
    GZ -. "ground truth, benchmark only" .-> B
```

### Robot and sensors

| Property | Value |
|---|---|
| Platform | Robotnik RB-Summit-XL Steel ([meshes](#third-party-assets)) |
| Chassis | 0.719 × 0.579 × 0.443 m, 66 mm ground clearance |
| Height with LiDAR | 0.695 m |
| Mass | 125.9 kg: 99 kg chassis, 4 × 6.5 kg wheels, 0.9 kg sensors |
| Wheels | 4 mecanum, 0.127 m radius, 0.099 m wide, 0.445 m wheelbase, 0.409 m track |
| Drive | Gazebo MecanumDrive, friction along the roller axis (`fdir1` at 45°, `mu2` = 0) |
| Limits | 1.5 m/s and 1.5 rad/s, 1 m/s² and 1 rad/s² |
| Collision | Primitives: box chassis, sphere wheels, cylinder mast and LiDAR |
| Inertia | Computed from the primitive shapes |

The robot fits 3 m aisles and the 0.9 m passage in aisle A3, with 0.16 m clear on
each side of the chassis and 0.12 m on each side of the wheel collision spheres.

| Sensor | Interface | Frame | Configuration |
|---|---|---|---|
| 3D LiDAR (`gpu_lidar`) | `/lidar/points` `PointCloud2` | `lidar_link`, 0.66 m above ground, body centre | 16 channels ±15°, 900 samples/rev (0.4°), 10 Hz, 0.3–30 m, σ = 1 cm |
| RGB camera | `/camera/image_raw`, `/camera/camera_info` | `camera_optical_frame`, 0.417 m above ground, front face | 640 × 480 `rgb8`, 20 Hz, 1.2 rad HFOV |
| IMU | `/imu/data` | `imu_link`, 0.288 m above ground, body centre | 100 Hz, MEMS-grade Gaussian noise and bias |
| Wheel odometry | `/odom`, TF `odom → base_footprint` | `base_footprint` | Gazebo MecanumDrive, integrated from wheel joint positions, 50 Hz |

### Perception pipeline

`wsl_perception` (C++, PCL) turns `/lidar/points` into obstacles:

```
raw cloud → validate → transform + ROI → voxel grid → ground removal → clusters
```

1. **Validate and extract**: reject empty clouds, missing frame, non-float or missing
   xyz, big-endian and inconsistent step/size layouts before reading a byte; drop
   NaN/Inf; range-filter from the sensor origin, all in one pass.
2. **Transform + ROI** into `base_footprint` via TF.
3. **Voxel grid** (0.08 m).
4. **Ground removal**: RANSAC plane constrained to ≤ 0.15 rad tilt, fitted only to
   points below 0.3 m so shelf decks cannot win.
5. **Euclidean clustering** (0.25 m) → axis-aligned boxes.

**Outputs:** `/perception/filtered_points` · `/perception/obstacle_points` ·
`/perception/clusters` (`vision_msgs/Detection3DArray`) · `/perception/markers` ·
`/diagnostics` (input rate, processing time, rejections).

Every parameter has a unit in its name and a valid range, and the full set is
re-validated when changed at runtime.

### SLAM pipeline

RTAB-Map graph SLAM ([`wsl_slam/config/rtabmap.yaml`](src/wsl_slam/config/rtabmap.yaml)):

- Wheel odometry read from TF at each scan's stamp
- Point-to-plane ICP between consecutive graph nodes
- Loop closure by scan proximity
- 3-DoF optimization
- 2D occupancy grid projected from the 3D scans

Publishes `map → odom`, `/rtabmap/map`, `/rtabmap/mapPath` and `/rtabmap/cloud_map`.

## Launch options

```bash
ros2 launch wsl_sim warehouse.launch.py headless:=true rviz:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `headless` | `false` | Gazebo server only; sensors still render offscreen |
| `rviz` | `true` | Start RViz (fixed frame `map` with SLAM, `odom` without) |
| `perception` | `true` | Start the point-cloud pipeline |
| `slam` | `true` | Start RTAB-Map |
| `ground_truth` | `false` | Publish `/ground_truth/odom` (evaluation only) |
| `world` | `warehouse` | World file in `wsl_sim/worlds` |
| `x`, `y`, `yaw` | `-12.0`, `9.0`, `0.0` | Spawn pose in the world frame |
| `lidar_channels`, `lidar_horizontal_samples`, `lidar_rate_hz` | `16`, `900`, `10` | LiDAR fidelity |
| `camera_rate_hz` | `20` | Camera frame rate |
| `odom_wheel_radius_scale`, `odom_wheel_separation_scale` | `1.0` | Odometry calibration error |

On machines without a GPU, Gazebo renders with Mesa; `headless:=true` uses `libegl-mesa0`.

## Benchmark

Run a scenario. The robot drives the route, then the launch exits:

```bash
ros2 launch wsl_benchmark benchmark.launch.py scenario:=warehouse_loop
```

Results are written to `results/warehouse_loop_<date>_<time>/` (raw CSVs and `metrics.json`).

| Scenario | Focus |
|---|---|
| `warehouse_loop` | Long loop, loop closure, odometry drift |
| `aisle_loop` | Repetitive rack geometry |
| `warehouse_loop_lowres` | Reduced LiDAR resolution |
| `manual` | Manual driving with full recording |

Recompute metrics from a recorded run:

```bash
ros2 run wsl_benchmark evaluate results/<run>
```

Benchmark the perception algorithms alone (no ROS graph, no simulator):

```bash
./build/wsl_perception/pipeline_benchmark 300
```

## Testing

```bash
colcon test && colcon test-result --verbose
```

| Suite | Tests |
|---|---|
| `wsl_perception` | GoogleTest: parser incl. hostile layouts, each stage, boundaries, config validation; randomized property tests; launch test of the real node |
| `wsl_benchmark` | pytest: metrics against analytic results, evaluation end to end, scenarios, pure pursuit |
| `wsl_description` | URDF structure, inertia physics, mesh checks (collisions match, roller direction, no z-fighting), LiDAR clearance, drive limits, frame conventions, `check_urdf` |
| `wsl_sim` | Asset and robot mesh resolution, bridge directions, ground-truth isolation, bridge ↔ robot topic consistency |

End-to-end smoke test (launches the full system headless and checks sensors, frames,
rates, TF authorities and driving):

```bash
colcon build --cmake-args -DWSL_SIM_TESTS=ON && colcon test --packages-select wsl_sim
```

| Flag | Enables |
|---|---|
| `-DWSL_SANITIZE=ON` | ASan + UBSan |
| `-DWSL_BUILD_FUZZERS=ON` | libFuzzer target for the PointCloud2 parser (clang) |

CI runs build, tests, sanitizers, clang-format, flake8 and the simulation smoke test
on amd64 and arm64.

## Supported platforms

| | |
|---|---|
| OS | Ubuntu 24.04 LTS (native) · macOS and Windows (dev container) |
| ROS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic (gz-sim 8) via `ros_gz` |
| SLAM | RTAB-Map (`rtabmap_slam`), PCL ICP backend |
| Languages | C++17 (perception), Python 3 (launch, benchmark) |
| Architectures | amd64 and arm64 |

## Third-party assets

The robot's visual meshes are not ours. They are Robotnik's, and the rest of the
project is Apache-2.0.

| | |
|---|---|
| Source | [RobotnikAutomation/robotnik_description](https://github.com/RobotnikAutomation/robotnik_description), branch `jazzy-devel`, commit `47966067b2105a84da1a07f06b3cc7c44706fe34` |
| Copyright | © 2025 Robotnik Automation S.L. |
| Licence | BSD-3-Clause, kept with the files: [`meshes/rbsummit_steel/LICENSE`](src/wsl_description/meshes/rbsummit_steel/LICENSE) |
| Files | `rbsummit_steel_chassis.stl`, `rbsummit_steel_top_cover.stl`, `kairos_mecanum_wheel_1.dae`, `kairos_mecanum_wheel_2.dae` |
| Modified | The files are byte-identical to upstream. In our URDF the top cover sits 1.5 mm higher than upstream to avoid z-fighting, colours are set from Robotnik's datasheet photo, and collision uses primitives instead of the chassis mesh. Robotnik's logo meshes and sensors are not used. |

## Documentation

| Document | Contents |
|---|---|
| [Architecture](docs/architecture.md) | Nodes, data flow, performance profiling |
| [Frames](docs/frames.md) | TF tree and frame conventions |
| [Benchmarking](docs/benchmarking.md) | Metrics and methodology |

---

<div align="center">

© 2026 Azer Ben Ali · [Apache-2.0](LICENSE) · v0.1.0

</div>
