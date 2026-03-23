# MoveIt 2 Surface Wiping Demo

This repository implements a countertop wiping workflow for a 6-DOF arm in MoveIt 2. The delivered scope covers the minimum required pipeline for the countertop task: scene setup, collision-aware IK reachability analysis, raster coverage planning, and a simulated contact-aware wiping controller with logging and RViz replay support.

The robot used for validation is `ur5e` with the Universal Robots Jazzy stack.

## Highlights

- countertop reachability analysis over a `60 x 60 cm` patch at `2 cm` resolution
- raster wiping plan with `100 x 50 mm` pad, `15 mm` keep-out margin, and `15%` overlap
- contact-aware simulated wiping controller with force-threshold switching and safety backoff
- RViz replay markers and a recorded demo video

## Demo

Video:

![Section 3 Demo Video](docs/assets/demo.gif)

Reachability heatmap:

![Reachability Heatmap](docs/assets/reachability_heatmap.png)

Coverage plan:

![Coverage Plan](docs/assets/coverage_plan.png)

Force and velocity tracking:

![Force and Velocity Tracking](docs/assets/wiping_force_velocity.png)

## Scope

Implemented:

- Section 1: countertop scene setup, IK service, 60 x 60 cm reachability map at 2 cm resolution
- Section 2: raster coverage planning for the countertop with keep-out margin and overlap
- Section 3: simulated contact-aware wiping control, force/velocity logging, RViz replay markers

Not part of the delivered minimum scope:

- mirror wiping strategy
- spiral coverage strategy
- real force sensor or hardware force control
- full local replanning around the faucet

## Section 1: Kinematics and Reachability

Implemented:

- countertop, faucet, and mirror collision objects in the planning scene
- a custom `reachability_query` service backed by MoveIt `/compute_ik`
- collision-aware IK checks for surface-aligned countertop targets
- reachability sampling over a `60 x 60 cm` patch at `2 cm` resolution
- CSV export and heatmap generation

Latest artifacts:

- `outputs/reachability_map_20260322_213355.csv`
- `outputs/reachability_heatmap_20260322_213355.png`

Current result summary:

- sampled points: `961`
- reachable points: `383`
- reachable ratio: `39.85%`
- dominant failure mode: `ik_failed` on `517` points
- bounds-rejected points: `61`

Interpretation:

- the central countertop region is reachable more often than the boundary
- the fixed wiping orientation reduces valid IK solutions
- collision-aware IK rejects solutions that would intersect the planning scene

## Section 2: Surface Coverage Path Planning

Implemented:

- raster coverage strategy for the countertop
- tool footprint parameterization: `100 x 50 mm`
- keep-out margin: `15 mm`
- overlap: `15%`
- reachable-cell filtering using Section 1 outputs
- waypoint CSV, joint-state sequence CSV, metrics CSV, and planner plot

Latest artifacts:

- `outputs/coverage_waypoints_20260323_204840.csv`
- `outputs/joint_trajectory_20260323_204840.csv`
- `outputs/coverage_metrics_20260323_204840.csv`
- `outputs/coverage_plan_20260323_204840.png`

Current result summary:

- reachable waypoints: `191`
- waypoint x-range: `0.32 m` to `0.74 m`
- waypoint y-range: `-0.28 m` to `0.28 m`
- coverage: `76.22%`
- path length: `6.27 m`
- estimated execution time: `1702.68 s`

Design note:

The joint trajectory output is a joint-state sequence with simple timing estimation from inter-waypoint joint deltas. It is suitable for evaluation and reporting, but it is not a full MoveIt time-parameterized execution trajectory.

## Section 3: Contact-Aware Wiping Control

Implemented:

- a simulated wiping controller with `APPROACH`, `CONTACT`, and `BACKOFF` modes
- switch to contact mode when `|Fz| > 2 N`
- target countertop force tracking at `10 N`
- target countertop tangential speed at `0.20 m/s`
- safety backoff when `|Fz| > 15 N`
- obstacle handling by skipping faucet keep-out waypoints
- force and velocity logging
- RViz replay markers for demo recording

Latest artifacts:

- `outputs/wiping_log_20260323_204959.csv`
- `outputs/wiping_metrics_20260323_204959.csv`
- `outputs/wiping_force_velocity_20260323_204959.png`
- `outputs/Screencast from 2026-03-23 20-55-55.webm`

Current result summary:

- surface: `countertop`
- target force: `10.0 N`
- target speed: `0.20 m/s`
- simulated duration: `31.9 s`
- total waypoints: `191`
- skipped waypoints: `0`
- contact switches: `1`
- backoff events: `0`

Interpretation:

- the RViz replay shows the wiping path, controller state text, and current TCP replay marker
- the plot shows force regulation and tangential speed over time
- together, the replay and plots satisfy the requested "video/gif or sim run" deliverable

## Build

```bash
cd /home/sihan/ws_wiping
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select moveit2_surface_wiping_demo moveit2_surface_wiping_interfaces
source install/setup.bash
```

## Run With UR5e

Terminal 1:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.56.101 \
    use_mock_hardware:=true \
    launch_rviz:=false
```

Terminal 2:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true
```

Section 1:

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo demo.launch.py \
    planning_group:=ur_manipulator \
    end_effector_link:=tool0
```

Section 2:

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo coverage.launch.py \
    planning_group:=ur_manipulator \
    end_effector_link:=tool0
```

Section 3:

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo control.launch.py
```

Section 3 RViz replay:

Add two `MarkerArray` displays in RViz:

- `/wiping_demo_static_markers`
- `/wiping_demo_markers`

Then run:

```bash
source /home/sihan/ws_wiping/install/setup.bash
ros2 launch moveit2_surface_wiping_demo control_viz.launch.py playback_rate:=1.0
```

## Parameters

Key parameters are defined in:

- `src/moveit2_surface_wiping_demo/moveit2_surface_wiping_demo/config/scene.yaml`
- `src/moveit2_surface_wiping_demo/moveit2_surface_wiping_demo/config/reachability.yaml`
- `src/moveit2_surface_wiping_demo/moveit2_surface_wiping_demo/config/coverage.yaml`
- `src/moveit2_surface_wiping_demo/moveit2_surface_wiping_demo/config/control.yaml`

Important values:

- patch size: `0.60 x 0.60 m`
- reachability resolution: `0.02 m`
- pad size: `0.10 x 0.05 m`
- keep-out margin: `0.015 m`
- overlap: `0.15`
- countertop target force: `10 N`
- countertop target speed: `0.20 m/s`

## Limitations and Trade-Offs

- The delivered task scope is countertop wiping only. The mirror is modeled in the scene but is not implemented as a full wiping target.
- Faucet and mirror are included in the MoveIt planning scene, so they affect collision-aware IK reachability queries. The coverage planner itself does not perform explicit 2D obstacle-footprint carving on the wiping surface.
- Obstacle handling in Section 3 is waypoint skipping around the faucet keep-out region, not local replanning.
- The RViz replay shows the simulated controller state using markers; it does not drive the UR5e joints through the wiping log.
- The joint trajectory output is an evaluation artifact derived from IK samples, not a full execution-ready trajectory from a motion planner.

## Files

The package source and package-level documentation remain under:

- `src/moveit2_surface_wiping_demo/moveit2_surface_wiping_demo/`
