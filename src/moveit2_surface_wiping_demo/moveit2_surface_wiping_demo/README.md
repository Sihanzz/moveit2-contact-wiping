# MoveIt 2 Surface Wiping Demo

This package implements a countertop wiping workflow for a 6-DOF arm in MoveIt 2. The delivered scope is the minimum required pipeline for the countertop task: scene setup, collision-aware IK reachability analysis, raster coverage planning, and a simulated contact-aware wiping controller with logging and RViz replay support.

The current robot setup used for validation is `ur5e` with the Universal Robots Jazzy stack.

## Highlights

- countertop reachability analysis over a `60 x 60 cm` patch at `2 cm` resolution
- raster wiping plan with `100 x 50 mm` pad, `15 mm` keep-out margin, and `15%` overlap
- contact-aware simulated wiping controller with force-threshold switching and safety backoff
- RViz replay markers plus a recorded demo video in `outputs/`

## Demo Assets

Latest visual assets:

- video demo: `outputs/Screencast from 2026-03-23 20-55-55.webm`
- reachability heatmap: `outputs/reachability_heatmap_20260322_213355.png`
- coverage plan: `outputs/coverage_plan_20260323_204840.png`
- force and velocity tracking: `outputs/wiping_force_velocity_20260323_204959.png`

### Video Demo

![Demo](../../../outputs/demo.gif)

### Reachability Heatmap

![Reachability Heatmap](../../../outputs/reachability_heatmap_20260322_213355.png)

### Coverage Plan

![Coverage Plan](../../../outputs/coverage_plan_20260323_204840.png)

### Force and Velocity Tracking

![Force and Velocity Tracking](../../../outputs/wiping_force_velocity_20260323_204959.png)

## Scope

Implemented:

- Section 1: countertop scene setup, IK service, 60 x 60 cm reachability map at 2 cm resolution
- Section 2: raster coverage planning for the countertop with keep-out margin and overlap
- Section 3: simulated contact-aware wiping control, force/velocity logging, RViz replay markers

Not part of the delivered minimum scope:

- mirror wiping strategy
- spiral coverage strategy
- real force sensor / hardware force control
- full local replanning around the faucet

## Repository Layout

- `moveit2_surface_wiping_demo/scene_setup.py`
  Publishes countertop, faucet, and mirror collision objects.
- `moveit2_surface_wiping_demo/ik_service.py`
  Wraps MoveIt `/compute_ik` as a simpler reachability query service for surface-aligned countertop targets.
- `moveit2_surface_wiping_demo/reachability_map.py`
  Samples the countertop patch, checks IK feasibility, exports CSV, and saves a heatmap.
- `moveit2_surface_wiping_demo/coverage_planner.py`
  Builds a reachable raster path and exports waypoint, metrics, and joint-sequence CSVs.
- `moveit2_surface_wiping_demo/wiping_controller.py`
  Simulates contact switching, force tracking, and safety backoff during wiping.
- `moveit2_surface_wiping_demo/wiping_visualization.py`
  Replays the latest wiping log in RViz using markers.

## Planning Scene

The scene is defined in `config/scene.yaml` and contains:

- countertop: `1.20 x 0.60 x 0.04 m`
- faucet obstacle: `0.12 x 0.05 x 0.30 m`
- mirror obstacle: `0.90 x 0.02 x 0.60 m`

All objects are published in the `base_link` frame.

## Section 1: Kinematics and Reachability

### What is implemented

- collision objects for countertop, faucet, and mirror
- a custom `reachability_query` service backed by MoveIt `/compute_ik`
- collision-aware IK checking
- 60 x 60 cm countertop patch sampling at 2 cm resolution
- CSV export and heatmap generation

### IK behavior

The IK service accepts a target point on the configured countertop patch and tests a surface-aligned end-effector pose with collision checking enabled. It rejects requests when:

- the point is outside the configured patch bounds
- the requested `z` does not match the configured countertop wiping height
- MoveIt cannot find a collision-free IK solution

### Reachability result summary

Latest reachability artifacts:

- `outputs/reachability_map_20260322_213355.csv`
- `outputs/reachability_heatmap_20260322_213355.png`

Measured result from the current output:

- sampled points: `961`
- reachable points: `383`
- reachable ratio: `39.85%`
- dominant failure mode: `ik_failed` on `517` points
- bounds-rejected points: `61`

![Reachability Result](../../../outputs/reachability_heatmap_20260322_213355.png)

### Reachability analysis

Where the arm can reach:

- the central countertop region is reachable more often than the patch boundary
- reachable samples extend roughly from `x = 0.30 m` to `0.74 m`
- reachable samples extend roughly from `y = -0.30 m` to `0.28 m`

Where the arm cannot reach well:

- near patch edges, especially far boundary regions
- at samples where the required surface-aligned wrist orientation creates poor kinematic conditioning
- at samples where collision-aware IK fails because of arm posture or scene obstacles

Why:

- the UR5e workspace is not symmetric over the sampled patch
- the wiping pose fixes the tool orientation, which reduces the set of valid IK solutions
- collision checking removes otherwise valid geometric solutions

## Section 2: Surface Coverage Path Planning

### What is implemented

- raster coverage strategy for the countertop
- pad footprint parameterization: `100 x 50 mm`
- keep-out margin: `15 mm`
- overlap: `15%`
- reachable-cell filtering using Section 1 outputs
- waypoint CSV, joint-state sequence CSV, planner plot, and summary metrics

### Planning method

The planner first constructs a feasible wiping mask from reachable cells. It then trims the mask using the keep-out margin so the pad center does not touch the countertop edge. A raster path is generated over the trimmed region using alternating sweep directions.

The planner exports:

- `coverage_waypoints_*.csv`
- `joint_trajectory_*.csv`
- `coverage_metrics_*.csv`
- `coverage_plan_*.png`

### Current coverage result summary

Latest coverage artifacts:

- `outputs/coverage_waypoints_20260323_204840.csv`
- `outputs/joint_trajectory_20260323_204840.csv`
- `outputs/coverage_metrics_20260323_204840.csv`
- `outputs/coverage_plan_20260323_204840.png`

Measured result from the current output:

- reachable waypoints: `191`
- waypoint x-range: `0.32 m` to `0.74 m`
- waypoint y-range: `-0.28 m` to `0.28 m`
- coverage: `76.22%`
- path length: `6.27 m`
- estimated execution time: `1702.68 s`

![Coverage Result](../../../outputs/coverage_plan_20260323_204840.png)

### Design note

The joint trajectory output is a joint-state sequence with simple timing estimation from inter-waypoint joint deltas. It is suitable for evaluation and reporting, but it is not a full MoveIt time-parameterized execution trajectory.

## Section 3: Contact-Aware Wiping Control

### What is implemented

- a simulated wiping controller with three modes:
  - `APPROACH`
  - `CONTACT`
  - `BACKOFF`
- switch to contact mode when `|Fz| > 2 N`
- target countertop force tracking at `10 N`
- target countertop tangential speed at `0.20 m/s`
- safety backoff when `|Fz| > 15 N`
- obstacle handling by skipping faucet keep-out waypoints
- force and velocity logging
- force/velocity plot generation
- RViz replay markers for demo recording

### Control model

The controller uses a simple simulated contact model:

- contact force is estimated from penetration into the countertop plane
- XY motion follows the planned path
- Z velocity is adjusted to reduce force error during contact
- if force exceeds the configured safety limit, the controller backs off before continuing

This is a software-only contact simulation, not a physics-engine contact model and not a hardware force controller.

### Demo interpretation

The Section 3 video should be read together with the tracking plot:

- the RViz replay shows the wiping path, current TCP replay marker, and controller state text
- the force plot shows contact switching and force regulation behavior over time
- the velocity plot shows the tangential wiping speed profile during the simulated run

Together, these artifacts satisfy the requested "video/gif or sim run" deliverable without claiming real hardware contact execution.

### Current control result summary

Latest control artifacts:

- `outputs/wiping_log_20260323_204959.csv`
- `outputs/wiping_metrics_20260323_204959.csv`
- `outputs/wiping_force_velocity_20260323_204959.png`
- `outputs/Screencast from 2026-03-23 20-55-55.webm`

Measured result from the current output:

- surface: `countertop`
- target force: `10.0 N`
- target speed: `0.20 m/s`
- simulated duration: `31.9 s`
- total waypoints: `191`
- skipped waypoints: `0`
- contact switches: `1`
- backoff events: `0`

![Control Result](../../../outputs/wiping_force_velocity_20260323_204959.png)

## Build

```bash
cd /home/sihan/ws_wiping
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select moveit2_surface_wiping_demo moveit2_surface_wiping_interfaces
source install/setup.bash
```

## Run With UR5e

### Terminal 1: UR driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.56.101 \
    use_mock_hardware:=true \
    launch_rviz:=false
```

### Terminal 2: MoveIt and RViz

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true
```

### Section 1: scene plus reachability

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo demo.launch.py \
    planning_group:=ur_manipulator \
    end_effector_link:=tool0
```

### Section 2: scene plus coverage planning

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo coverage.launch.py \
    planning_group:=ur_manipulator \
    end_effector_link:=tool0
```

### Section 3: simulated wiping controller

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo control.launch.py
```

### Section 3 demo replay in RViz

Add two `MarkerArray` displays in RViz:

- `/wiping_demo_static_markers`
- `/wiping_demo_markers`

Then run:

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo control_viz.launch.py playback_rate:=1.0
```

## Parameters

Key parameters are kept in config files:

- `config/scene.yaml`
  scene geometry
- `config/reachability.yaml`
  patch size, resolution, surface offset
- `config/coverage.yaml`
  pad size, overlap, keep-out margin, raster settings
- `config/control.yaml`
  contact thresholds, target force, speed, backoff distance

Important task values currently used:

- patch size: `0.60 x 0.60 m`
- reachability resolution: `0.02 m`
- pad size: `0.10 x 0.05 m`
- keep-out margin: `0.015 m`
- overlap: `0.15`
- countertop target force: `10 N`
- countertop target speed: `0.20 m/s`

## Outputs to Show in Final Submission

Recommended final artifacts to present:

- `outputs/Screencast from 2026-03-23 20-55-55.webm`
- `outputs/reachability_map_20260322_213355.csv`
- `outputs/reachability_heatmap_20260322_213355.png`
- `outputs/coverage_waypoints_20260323_204840.csv`
- `outputs/joint_trajectory_20260323_204840.csv`
- `outputs/coverage_metrics_20260323_204840.csv`
- `outputs/coverage_plan_20260323_204840.png`
- `outputs/wiping_log_20260323_204959.csv`
- `outputs/wiping_metrics_20260323_204959.csv`
- `outputs/wiping_force_velocity_20260323_204959.png`

For the demo video, show:

- UR5e in RViz with scene objects visible
- planned countertop raster path
- Section 3 replay markers from `control_viz.launch.py`
- force/velocity tracking plot as supporting evidence

## Limitations and Trade-Offs

- The delivered task scope is countertop wiping only. The mirror is modeled in the scene but is not implemented as a full wiping target.
- Faucet and mirror are included in the MoveIt planning scene, so they affect collision-aware IK reachability queries. The coverage planner itself does not perform explicit 2D obstacle-footprint carving on the wiping surface.
- Obstacle handling in Section 3 is waypoint skipping around the faucet keep-out region, not local replanning.
- The RViz replay shows the simulated controller state using markers; it does not drive the UR5e joints through the wiping log.
- The joint trajectory output is an evaluation artifact derived from IK samples, not a full execution-ready trajectory from a motion planner.

## Conclusion

The project satisfies the minimum countertop-wiping requirements:

- collision scene configuration
- collision-aware IK reachability service
- 60 x 60 cm reachability heatmap at 2 cm resolution
- raster surface coverage planning with required tool footprint and margin parameters
- simulated contact-aware wiping controller with threshold switching, safety backoff, and tracking plots

The strongest final deliverables are the generated CSV and image artifacts plus the recorded RViz demo video in `outputs/Screencast from 2026-03-23 20-55-55.webm`.
