# Rover Path Following ROS2 Package

A complete ROS2 implementation for rover path following that only turns when stationary and merges smoothly onto reference paths when drifting away.

## 🚀 Features

- **Stationary Turning Only**: Rover stops completely before rotating for precise heading control
- **Smooth Merging**: Uses adaptive look-ahead distance proportional to cross-track error  
- **Configurable Parameters**: All control parameters tunable via ROS2 parameters
- **Timer-Based Control**: Runs at configurable frequency (default 10Hz)
- **Easy Integration**: Subscribes to standard `/values` topic for sensor fusion

## 📁 Package Structure

```
rover_navigation/
├── rover_navigation/
│   ├── __init__.py                      # Package initialization
│   └── rover_path_follower.py           # Main path following node
├── launch/
│   └── rover_path_follower_launch.py    # Launch file with parameters
├── config/
│   └── rover_config.yaml               # Configuration file
├── test/
│   └── test_values_publisher.py        # Test data publisher
├── package.xml                         # Package metadata
├── setup.py                           # Python package setup
├── README.md                          # This file
└── resource/
    └── rover_navigation                # Resource marker
```

## 🛠️ Installation

### 1. Prerequisites
- ROS2 Humble
- Python 3.8+
- NumPy

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-numpy
```

### 2. Create Workspace (if needed)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Copy Package Files
```bash
# Copy all the package files to ~/ros2_ws/src/rover_navigation/
# Make sure the directory structure matches the layout above
```

### 4. Create Resource Directory
```bash
cd ~/ros2_ws/src/rover_navigation
mkdir -p resource
touch resource/rover_navigation
```

### 5. Create Launch and Config Directories  
```bash
mkdir -p launch config
# Move rover_path_follower_launch.py to launch/
# Move rover_config.yaml to config/
```

### 6. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select rover_navigation
source install/setup.bash
```

## 🎯 Algorithm Overview

The rover implements your exact specifications:

1. **Calculate cross-track error (CTE)** from the reference line
2. **Project rover position** onto line to find closest point (t0)
3. **Choose look-ahead distance** L = max(L_min, k × |CTE|)  
4. **Compute target point** on line at t0 + L
5. **Calculate heading** from rover to target point
6. **Turn in place** until heading matches target (within tolerance)
7. **Move forward** to merge with line at target point
8. **When CTE ≤ tolerance**, turn to match reference heading and continue

## 🚀 Usage

### Basic Usage
```bash
# Terminal 1 - Run the path follower
ros2 run rover_navigation rover_path_follower

# Terminal 2 - Publish test data (for testing)
ros2 run rover_navigation test_values_publisher
```

### Using Launch File
```bash
ros2 launch rover_navigation rover_path_follower_launch.py
```

### With Custom Parameters
```bash
ros2 launch rover_navigation rover_path_follower_launch.py \
    L_min:=3.0 \
    k:=2.0 \
    CTE_threshold:=0.5 \
    forward_velocity:=1.5 \
    control_frequency:=20.0
```

### Using Configuration File
```bash
ros2 launch rover_navigation rover_path_follower_launch.py \
    --ros-args --params-file config/rover_config.yaml
```

## ⚙️ Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `L_min` | 2.0 | m | Minimum look-ahead distance |
| `k` | 1.5 | - | Cross-track error multiplier |
| `CTE_threshold` | 0.3 | m | When to start correction |
| `CTE_tolerance` | 0.1 | m | When correction is complete |
| `heading_tol` | 0.05 | rad | Heading alignment tolerance (~3°) |
| `forward_velocity` | 1.0 | m/s | Forward movement speed |
| `angular_velocity` | 0.5 | rad/s | Turning speed |
| `control_frequency` | 10.0 | Hz | Control loop frequency |
| `path_start_x` | 0.0 | m | Reference path start X |
| `path_start_y` | 0.0 | m | Reference path start Y |
| `path_end_x` | 20.0 | m | Reference path end X |
| `path_end_y` | 0.0 | m | Reference path end Y |

## 📡 Topics

### Subscribed Topics
- `/values` (std_msgs/Float64MultiArray): Fused GPS+IMU+Odometry data
  - `data[0]`: X position (meters)
  - `data[1]`: Y position (meters)
  - `data[2]`: Heading (radians)

### Published Topics  
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for the rover
  - `linear.x`: Forward velocity (m/s)
  - `angular.z`: Angular velocity (rad/s)

## 🧪 Testing

### Test with Simulated Data
```bash
# Terminal 1 - Start the path follower
ros2 run rover_navigation rover_path_follower

# Terminal 2 - Publish simulated rover data (starts off-path)
ros2 run rover_navigation test_values_publisher

# Terminal 3 - Monitor velocity commands
ros2 topic echo /cmd_vel

# Terminal 4 - Monitor sensor data
ros2 topic echo /values
```

### Monitor Status
```bash
# View log output
ros2 topic echo /rosout | grep rover_path_follower

# Check node info
ros2 node info /rover_path_follower

# List parameters
ros2 param list /rover_path_follower
```

### Visualize in RViz (Optional)
If you have visualization needs, you can add markers to show the path, rover position, and target points.

## 🔧 Customization

### Change Reference Path
Modify parameters in launch file or config file:
```yaml
path_start_x: 5.0    # New start point
path_start_y: 2.0
path_end_x: 25.0     # New end point  
path_end_y: 8.0
```

### Tune Control Parameters
For different rover characteristics:
```bash
# Faster, more aggressive
ros2 param set /rover_path_follower forward_velocity 2.0
ros2 param set /rover_path_follower angular_velocity 1.0

# Slower, more precise
ros2 param set /rover_path_follower forward_velocity 0.5
ros2 param set /rover_path_follower CTE_tolerance 0.05
```

## 🐛 Troubleshooting

### Common Issues

1. **"No /values data received"**
   ```bash
   # Check if /values topic exists
   ros2 topic list | grep values

   # Check message format
   ros2 topic echo /values
   ```

2. **Rover not moving**
   ```bash
   # Check /cmd_vel output
   ros2 topic echo /cmd_vel

   # Verify parameters
   ros2 param get /rover_path_follower forward_velocity
   ```

3. **Jerky movement**
   ```bash
   # Reduce angular velocity
   ros2 param set /rover_path_follower angular_velocity 0.3

   # Increase heading tolerance
   ros2 param set /rover_path_follower heading_tol 0.1
   ```

4. **Overshooting path**
   ```bash
   # Reduce forward velocity
   ros2 param set /rover_path_follower forward_velocity 0.8

   # Tighten CTE tolerance
   ros2 param set /rover_path_follower CTE_tolerance 0.05
   ```

### Debug Commands
```bash
# View all rover parameters
ros2 param dump /rover_path_follower

# Real-time parameter monitoring
ros2 run rqt_reconfigure rqt_reconfigure

# Performance monitoring  
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z
```

## 📊 Expected Behavior

1. **Normal Operation**: Rover moves forward along reference path
2. **CTE Detection**: When drift exceeds threshold, rover stops
3. **Turn to Target**: Rover rotates in place to face merge point  
4. **Merge Phase**: Rover moves forward toward path
5. **Final Alignment**: Rover aligns with reference heading
6. **Resume**: Normal path following continues

## 📝 Implementation Notes

- Uses **continuous control loop** instead of state machine for simplicity
- **Boolean flags** track correction phases
- **Timer-based execution** ensures consistent timing
- **Stationary turning only** for precise heading control
- **Geometric calculations** for accurate path projection

## 🤝 Contributing

To extend this package:
1. Add new path types (curved, multi-segment)
2. Implement obstacle avoidance
3. Add visualization markers
4. Create unit tests

## 📄 License

MIT License - Feel free to use and modify for your projects.

---

**Need help?** Check the troubleshooting section or examine the log output for detailed status information.
