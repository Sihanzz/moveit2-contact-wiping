# MoveIt2 Surface Wiping Demo

A ROS2 package demonstrating robotic arm workspace analysis for surface wiping tasks using MoveIt2.

## Overview

This package evaluates the reachability and motion planning capabilities of a 6-DOF robotic arm for surface wiping applications. It implements inverse kinematics testing, collision avoidance, and workspace reachability analysis.

## Features

- **Scene Setup**: Configurable collision environment (countertop, obstacles)
- **IK Testing**: Service-based inverse kinematics validation with collision checking
- **Reachability Analysis**: Systematic workspace evaluation with heatmap generation
- **CSV Export**: Structured data output for post-processing
- **Visualization**: Automatic heatmap generation

## Package Contents

### Nodes

1. **scene_setup**
   - Publishes collision objects to MoveIt planning scene
   - Loads environment from `config/scene.yaml`
   - Objects: countertop, faucet, mirror

2. **ik_test**
   - IK solver interface with collision checking
   - Validates end-effector pose reachability
   - Configurable planning group and timeout

3. **reachability_map**
   - Generates workspace reachability heatmap
   - Grid-based pose sampling
   - Exports CSV + PNG visualization

### Configuration Files

- `config/scene.yaml`: Collision object definitions
- `config/reachability.yaml`: Analysis parameters

### Launch Files

- `launch/demo.launch.py`: Launches full analysis pipeline

## Dependencies

### ROS2 Packages
- `rclpy`: ROS2 Python client
- `moveit_msgs`: MoveIt message definitions
- `geometry_msgs`: Pose/transform messages
- `shape_msgs`: Collision shape definitions
- `sensor_msgs`: Joint state messages
- `tf2_ros`: Transform library

### Python Packages
- `numpy`: Numerical operations
- `matplotlib`: Visualization
- `pyyaml`: Configuration parsing

### System Requirements
- ROS2 (Humble or later)
- MoveIt2
- Robot description (URDF/SRDF)
- MoveIt configuration package for your robot

## Installation

```bash
# Navigate to workspace
cd ~/ws_wiping/src

# Clone or ensure package exists
# (Package already present in your workspace)

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build
cd ~/ws_wiping
colcon build --packages-select moveit2_surface_wiping_demo

# Source
source install/setup.bash
```

## Usage

### Prerequisites

Ensure MoveIt2 is running with your robot:

```bash
# Terminal 1: Launch MoveIt2 with your robot
ros2 launch <your_robot>_moveit_config demo.launch.py

# Example for Piper robot:
ros2 launch piper_moveit_config demo.launch.py
```

Verify the `/compute_ik` service is available:
```bash
ros2 service list | grep compute_ik
```

### Running the Demo

```bash
# Terminal 2: Launch the analysis
ros2 launch moveit2_surface_wiping_demo demo.launch.py

# With custom output directory:
ros2 launch moveit2_surface_wiping_demo demo.launch.py output_dir:=/home/user/results

# With custom planning group:
ros2 launch moveit2_surface_wiping_demo demo.launch.py \
    planning_group:=manipulator \
    end_effector_link:=tool0
```

### Running Nodes Individually

```bash
# Scene setup only
ros2 run moveit2_surface_wiping_demo scene_setup

# Reachability analysis only (requires scene setup first)
ros2 run moveit2_surface_wiping_demo reachability_map \
    --ros-args -p output_dir:=/tmp \
    -p planning_group:=piper_arm \
    -p end_effector_link:=link6
```

## Configuration

### Scene Configuration (`config/scene.yaml`)

Define collision objects in the environment:

```yaml
world_frame: base_link

countertop:
  id: countertop
  size: [1.20, 0.60, 0.04]  # X, Y, Z dimensions (m)
  position: [0.60, 0.20, 0.20]  # X, Y, Z center (m)

faucet:
  id: faucet
  size: [0.12, 0.05, 0.30]
  position: [0.55, 0.20, 0.37]

mirror:
  id: mirror
  size: [0.90, 0.02, 0.60]
  position: [0.75, -0.31, 0.55]
```

### Reachability Configuration (`config/reachability.yaml`)

Set analysis parameters:

```yaml
surface: countertop  # Which surface to analyze

patch_center: [0.60, 0.00]  # XY center of analysis region (m)
patch_size: [0.60, 0.60]    # XY dimensions of region (m)
resolution: 0.02             # Grid spacing (m) - 2cm

surface_z_offset: 0.02       # Height above surface (m)
approach_offset: 0.10        # Clearance for approach (m)

tool_normal_axis: z          # Tool axis that should point to surface
normal_tolerance_deg: 10.0   # Orientation tolerance
```

## Outputs

### CSV File Format

`reachability_map_YYYYMMDD_HHMMSS.csv`:
```csv
x, y, z, reachable
0.30, -0.30, 0.24, 0
0.32, -0.30, 0.24, 0
...
0.60, 0.00, 0.24, 1
```

- `x, y, z`: 3D position of test pose (m)
- `reachable`: 1 if IK solution exists and is collision-free, 0 otherwise

### Heatmap Visualization

`reachability_heatmap_YYYYMMDD_HHMMSS.png`:
- Color-coded visualization of workspace
- Green: Reachable positions
- Yellow: Transition zones
- Red: Unreachable positions

## Customization

### Using with Different Robots

Update launch arguments:
```bash
ros2 launch moveit2_surface_wiping_demo demo.launch.py \
    planning_group:=arm \
    end_effector_link:=tcp
```

Or modify parameters in the launch file:
```python
planning_group_arg = DeclareLaunchArgument(
    'planning_group',
    default_value='your_arm_group',  # Change here
    description='MoveIt planning group name'
)
```

### Adjusting Analysis Resolution

In `config/reachability.yaml`:
```yaml
resolution: 0.01  # 1cm grid - more detailed but 4x slower
# or
resolution: 0.05  # 5cm grid - faster but coarser
```

Grid size = (patch_size / resolution)²
- 2cm: 30×30 = 900 points (~10 min)
- 1cm: 60×60 = 3600 points (~40 min)
- 5cm: 12×12 = 144 points (~2 min)

## Troubleshooting

### "IK service not available"
- Ensure MoveIt's move_group node is running
- Check: `ros2 service list | grep compute_ik`
- Verify robot description is loaded

### "No collision objects published"
- Scene setup node may not have started
- Check: `ros2 topic echo /collision_object`
- Verify config file paths are correct

### Slow Performance
- Reduce grid resolution in config
- Decrease IK timeout in ik_test.py
- Use faster IK solver (TracIK instead of KDL)

### Matplotlib Errors
- Install: `pip3 install matplotlib`
- Or: `sudo apt install python3-matplotlib`

## Expected Results

For a typical 6-DOF arm:
- **Central workspace**: 40-60% reachable
- **Obstacle zones**: Low reachability near faucet
- **Boundary regions**: Limited reachability at edges
- **Runtime**: ~5-15 minutes for 900-point grid

See `REACHABILITY_ANALYSIS.md` in workspace root for detailed analysis.

## Development

### Testing Individual Components

```bash
# Test scene publishing
ros2 run moveit2_surface_wiping_demo scene_setup
ros2 topic echo /collision_object --once

# Test IK service manually
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "..."
```

### Extending Functionality

Add new nodes in `moveit2_surface_wiping_demo/`:
- Update `setup.py` entry_points
- Add to launch file
- Rebuild package

## References

- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [IK Solver Comparison](https://ros-planning.github.io/moveit_tutorials/)

## License

MIT

## Maintainer

Sihan Dong (sihandong14@gmail.com)

## Acknowledgments

Built for robotics workspace analysis and motion planning evaluation using MoveIt2 framework.
