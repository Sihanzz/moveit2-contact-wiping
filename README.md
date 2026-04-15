# Surface Wiping Assignment

UR5-based ROS 2 workspace for the three-part arm take-home assignment:
- Section 1: kinematics and reachability
- Section 2: surface coverage path planning
- Section 3: contact-aware wiping control

This repository is intentionally pragmatic. I kept the UR model unchanged, used `tool0` as the end-effector reference, and pushed contact-point compensation into configuration. That choice is less elegant than adding a dedicated `wiper_link`, but it kept the pipeline consistent across all three sections and made the assumptions explicit.

## Workspace

Packages:
- `surface_wiping_interfaces`: custom ROS interfaces
- `surface_wiping_nodes`: scene setup, IK server, reachability, planning, control
- `surface_wiping_bringup`: launch files and RViz config

Build:

```bash
cd /home/sihan/ws_surface_wiping_clean
colcon build --packages-select surface_wiping_interfaces surface_wiping_nodes surface_wiping_bringup
source install/setup.bash
```

Scene check:

```bash
ros2 launch surface_wiping_bringup scene_debug.launch.py
```

RViz:

```bash
source /home/sihan/ws_surface_wiping_clean/install/setup.bash
rviz2 -d /home/sihan/ws_surface_wiping_clean/install/surface_wiping_bringup/share/surface_wiping_bringup/rviz/scene_debug.rviz
```

Expected scene:
- `base_link` is the fixed frame
- countertop in front of the robot
- mirror behind the countertop
- faucet offset from the main wiping area

## Modeling Notes

Runtime TF inspection gives the following approximate axis relation:

- `tool0.x -> +base_link.x`
- `tool0.y -> -base_link.z`
- `tool0.z -> +base_link.y`
- `wrist_3_link -> tool0` is effectively identity

I use `tool0.z` as the wiping-face normal for both surfaces:
- countertop: `tool0.z -> -base_link.z`
- mirror: `tool0.z -> +base_link.x`

The basic orientation constraint is:


$$
\mathbf{n}_{tool} \approx \mathbf{n}_{surface}
$$

with a small angular tolerance for planning samples.

Because `tool0` is not the actual contact point, I apply a configuration-level offset:

$$
\mathbf{p}_{query} = \mathbf{p}_{surface} + d\,\mathbf{n}_{surface}
$$

where `d` is `surface_z_offset` or `contact_offset` depending on the section.  
This is the main modeling simplification in the project. My view is that this is acceptable for the assignment because it keeps the geometry and the trade-off explicit, while avoiding a larger URDF change late in the workflow.

## Section 1: Kinematics and Reachability

Run:

```bash
ros2 launch surface_wiping_bringup section1.launch.py
```

This launches:
- planning scene setup
- collision-aware IK service
- countertop reachability sampling

Artifacts:
- CSV: [reachability_map_20260415_170134.csv](/home/sihan/ws_surface_wiping_clean/outputs/reachability_map_20260415_170134.csv)
- Heatmap: [reachability_heatmap_20260415_170135.png](/home/sihan/ws_surface_wiping_clean/outputs/reachability_heatmap_20260415_170135.png)

![Countertop Reachability Heatmap](/outputs/reachability_heatmap_20260415_170135.png)

Sampling setup:
- patch size: `60 x 60 cm`
- resolution: `2 cm`
- total samples: `961`

Result summary:
- reachable: `608 / 961`
- reachability: `63.27%`
- dominant failure: `no_ik_solution` on `353` samples

Interpretation:
- the central and front countertop region is mostly reachable
- reachability drops near the back boundary of the patch
- isolated failures inside the reachable region are likely due to posture/collision filtering

The practical reachability test is binary:

$$
R(x,y)=
\begin{cases}
1, & \text{if collision-aware IK succeeds} \\
0, & \text{otherwise}
\end{cases}
$$

My main conclusion from Section 1 is that the countertop is reachable enough to support wiping, but only after modeling the contact offset carefully. Without that offset, the queried `tool0` poses are unrealistically close to the surface and IK collapses.

## Section 2: Surface Coverage Path Planning

Run:

```bash
ros2 launch surface_wiping_bringup section2.launch.py
```

Countertop uses a raster strategy. Mirror uses a spiral strategy.

### Countertop Raster

Artifacts:
- Waypoints: [coverage_waypoints_countertop_raster_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_waypoints_countertop_raster_20260415_183716.csv)
- Joint trajectory: [coverage_joint_trajectory_countertop_raster_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_joint_trajectory_countertop_raster_20260415_183716.csv)
- Metrics: [coverage_metrics_countertop_raster_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_metrics_countertop_raster_20260415_183716.csv)
- Plot: [coverage_plan_countertop_raster_20260415_183716.png](/home/sihan/ws_surface_wiping_clean/outputs/coverage_plan_countertop_raster_20260415_183716.png)

![Countertop Raster Plan](/outputs/coverage_plan_countertop_raster_20260415_183716.png)

Metrics:
- waypoints: `283`
- coverage: `78.61%`
- path length: `6.09 m`
- estimated execution time: `1161.56 s`

### Mirror Spiral

Artifacts:
- Waypoints: [coverage_waypoints_mirror_spiral_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_waypoints_mirror_spiral_20260415_183716.csv)
- Joint trajectory: [coverage_joint_trajectory_mirror_spiral_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_joint_trajectory_mirror_spiral_20260415_183716.csv)
- Metrics: [coverage_metrics_mirror_spiral_20260415_183716.csv](/home/sihan/ws_surface_wiping_clean/outputs/coverage_metrics_mirror_spiral_20260415_183716.csv)
- Plot: [coverage_plan_mirror_spiral_20260415_183716.png](/home/sihan/ws_surface_wiping_clean/outputs/coverage_plan_mirror_spiral_20260415_183716.png)

![Mirror Spiral Plan](/outputs/coverage_plan_mirror_spiral_20260415_183716.png)

Metrics:
- waypoints: `241`
- coverage: `66.94%`
- path length: `7.41 m`
- estimated execution time: `627.27 s`

Comparison:
- countertop: raster is the simpler and more defensible baseline on a rectangular patch
- mirror: spiral gives a smoother continuous path with fewer end-of-row reversals

Coverage is estimated from tool footprint and waypoint spacing:

$$
\text{coverage} \approx \frac{N \cdot \Delta s \cdot w_{pad}}{A_{patch}}
$$

where `N` is waypoint count, `Δs` is step size, `w_pad` is pad width, and `A_patch` is surface area.

My main design choice here was to let Section 2 inherit feasibility from Section 1 where possible. For the countertop, the planner consumes the reachability result directly. For the mirror, IK is checked online because the mirror surface is not part of the Section 1 heatmap. That split keeps the planner simple and avoids pretending the mirror has been pre-sampled when it has not.

## Section 3: Contact-Aware Wiping Control

Run:

```bash
ros2 launch surface_wiping_bringup section3.launch.py
```

This controller is simulation-grade rather than hardware-grade. It consumes the planned wiping path, simulates contact force evolution, switches mode at contact, applies backoff on excessive force, and logs force/velocity tracking.

### Countertop Control

Artifacts:

Demo video: ![countertop.webm](/outputs/countertop.gif)
- Metrics: [contact_wiping_metrics_countertop_20260415_224146.csv](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_metrics_countertop_20260415_224146.csv)
- Log: [contact_wiping_log_countertop_20260415_224146.csv](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_log_countertop_20260415_224146.csv)
- Tracking plot: [contact_wiping_tracking_countertop_20260415_224146.png](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_tracking_countertop_20260415_224146.png)

![Countertop Force and Velocity Tracking](/outputs/contact_wiping_tracking_countertop_20260415_224146.png)

Metrics:
- target force: `10 N ± 2 N`
- average force: `9.95 N`
- force within tolerance: `99.16%`
- average speed: `0.192 m/s`
- skipped waypoints near faucet: `23`
- backoff events: `0`

### Mirror Control

Artifacts:

Demo video: ![mirror.webm](/outputs/mirror.gif)
- Metrics: [contact_wiping_metrics_mirror_20260415_224146.csv](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_metrics_mirror_20260415_224146.csv)
- Log: [contact_wiping_log_mirror_20260415_224146.csv](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_log_mirror_20260415_224146.csv)
- Tracking plot: [contact_wiping_tracking_mirror_20260415_224146.png](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_tracking_mirror_20260415_224146.png)

![Mirror Force and Velocity Tracking](/outputs/contact_wiping_tracking_mirror_20260415_224146.png)

Metrics:
- target force: `6 N ± 1.5 N`
- average force: `5.98 N`
- force within tolerance: `99.63%`
- average speed: `0.144 m/s`
- skipped waypoints: `0`
- backoff events: `1`

Summary CSV:
- [contact_wiping_summary_20260415_224146.csv](/home/sihan/ws_surface_wiping_clean/outputs/contact_wiping_summary_20260415_224146.csv)

The control logic is a small state machine:
- `APPROACH`
- `FORCE`
- `BACKOFF`

The regulation idea is:

$$
F_{k+1} = F_k + K_f (F^* - F_k)\Delta t
$$

and if

$$
|F_z| > 15 \text{ N}
$$

the controller enters a backoff phase until force returns to a safe band.

Interpretation:
- countertop results meet the required force and speed bands and demonstrate obstacle skipping around the faucet
- mirror results meet the required force and speed bands and demonstrate one explicit safety backoff event
- the videos serve as the short sim-run demos requested in the assignment

My own view is that this section is strongest as a control-logic demonstration rather than a claim of full impedance control. The state transitions, target tracking, and safety behavior are all there, but the force is simulated rather than measured from a real wrist FT sensor. I think that is an honest and defensible boundary for the assignment.

## Design Choices and Trade-Offs

Main choices:
- keep the robot model unchanged and use `tool0`
- model contact-point offset in YAML
- use Section 1 reachability as a hard feasibility layer for the countertop
- use raster for countertop and spiral for mirror
- keep Section 3 as a simulated contact controller tied to Section 2 outputs

What I would improve next if I had more time:
- add a real `wiper_link` / TCP to remove some offset ambiguity
- replace simulated force with an actual FT topic and a proper low-level admittance or impedance loop
- add local replanning, not just skip logic, around the faucet
- make the Section 2 timing estimate a real time-parameterized execution trajectory

## Outputs

All generated artifacts are under:

```bash
/home/sihan/ws_surface_wiping_clean/outputs
```
