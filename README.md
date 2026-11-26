# Low-Level Obstacle Avoidance Using Depth Camera (ROS 2 Jazzy)

A ROS 2 Jazzy workspace focused on **detecting and avoiding low-level obstacles using a depth camera** on an Automated Guided Vehicle (AGV) in a Gazebo hospital environment (`aws_robomaker_hospital_world`).

Standard 2D LiDARs mounted at a fixed height often miss low-profile obstacles on the floor. This project uses a depth camera (`/depth_camera_1/points`) combined with height-filtering `PointCloud2`-to-`LaserScan` pipelines, Nav2 3D `VoxelLayer` / `ObstacleLayer` costmaps, `collision_monitor`, SLAM (`slam_toolbox` & `rtabmap_ros`), and frontier-based autonomous exploration.

![Simulation Output](src/asset/Output_video.gif)

---

## How Low-Level Obstacle Avoidance Works

1. **Depth Camera Sensing (`ros_gz_bridge`)**:
   - Bridges `/depth_camera_1/points` (`sensor_msgs/msg/PointCloud2`), `/camera/depth_image`, `/camera/image`, and `/camera/camera_info` from Gazebo Sim into ROS 2.
2. **Height-Filtered PointCloud-to-LaserScan Pipelines**:
   - `pointtofilter` (`pointtofilter.py`): Filters `/depth_camera_1/points` to isolate low-lying obstacles (`0.02 m < z < 0.20 m`) and publishes `/filtered_points`.
   - `flitertoscan` (`flitertoscan.py`): Converts `/filtered_points` into a 2D `LaserScan` on `/camera_scan`.
   - `cam_scan` (`pointtoscan.py`): Converts downward-facing `/depth_camera_1/points` (60° HFOV, `z` in `[-2.0, -0.05]`) directly to `/scan`.
   - `pointtoscan` (`pointcloudtolaserscan.py`): Filters `/rgbd_camera_1/points` (`0.0 m` to `0.3 m` height) to `/camera_scan`.
3. **Nav2 Costmap & Collision Avoidance (`config/navigation_2.yaml`)**:
   - **Local Costmap (`VoxelLayer`)** & **Global Costmap (`ObstacleLayer`)** subscribe to `/depth_camera_1/points` (`PointCloud2`) for 3D obstacle marking and raytrace clearing.
   - **`collision_monitor`** monitors `depth_cloud` (`/depth_camera_1/points`) to prevent collisions before executing `MPPIController` velocity commands.

---

## Workspace Structure

```text
ros2_ws/
├── myworld.data                          # Saved SLAM Toolbox map data
├── myworld.posegraph                     # Saved SLAM Toolbox pose graph
└── src/
    ├── agv_description/                  # AGV URDF/Xacro robot model & STL meshes
    ├── autonomous_navigation/            # Launch files, Nav2/SLAM configs, & Python nodes
    ├── aws_robomaker_hospital_world/     # Gazebo hospital worlds, 3D models, & assets
    └── asset/                            # Demo media (Output_video.gif)
```

---

## Packages Overview

### 1. `agv_description`
Contains the robot description, meshes, and visualization launch file for the **AGV-OTA** differential-drive robot (and `tortoisebot.urdf`):
- **URDF/Xacro** (`urdf/`):
  - `agv.urdf.xacro` — Main robot assembly (`agv_base`, `agv_wheel`, `agv_caster`, `agv_ir`, `agv_plugin`, `inertia_tensors`)
  - `agv_base.urdf.xacro` — Base chassis, `gpu_lidar`, `rgbd_camera`, and `DiffDrive` plugin
  - `agv_ir.urdf.xacro` — 8-sensor IR range array (`ir1`–`ir8`)
  - `agv_plugin.urdf.xacro` — Gazebo `JointStatePublisher`, `DiffDrive`, and `Imu` macros
  - `tortoisebot.urdf` — Standalone URDF with `depth_camera` (`rgbd_camera`), `gpu_lidar`, and `DiffDrive`
- **Meshes** (`meshes/OTAv07_meshes/`): STL models for the chassis, wheels, casters, photoelectric sensors, ZED camera, and S30B LiDAR.
- **Launch** (`launch/display.launch.py`): Spawns `agv.urdf.xacro` in `hospital.world` with `robot_state_publisher`, `joint_state_publisher`, `ros_gz_bridge`, and `rviz2`.

### 2. `autonomous_navigation`
Provides simulation bringup, SLAM, 3D mapping, Nav2 navigation, point cloud processing, and frontier exploration.

#### Launch Files (`launch/`)
| Launch File | Description |
| :--- | :--- |
| `display.launch.py` | Launches Gazebo (`gz sim -r myworld.world`), `robot_state_publisher`, spawns `agv`, starts `ros_gz_bridge`, and opens `rviz2`. |
| `slam.launch.py` | Launches `slam_toolbox` (`online_async_launch.py`) using `config/slam_toolbox_params.yaml` (`use_sim_time:=true`). |
| `nav2_launch.py` | Launches `nav2_bringup` (`navigation_launch.py`) using `config/navigation_2.yaml` (`use_sim_time:=true`). |
| `rtabmap.launch.py` | Launches `rtabmap_launch` (`rtabmap.launch.py`) for RGB-D 3D SLAM and assembled point cloud mapping (`Grid/3D:=true`, `Cloud/Assemble:=true`). |

#### ROS 2 Nodes (`console_scripts`)
| Executable | Source File | Description |
| :--- | :--- | :--- |
| `explore` | `explore.py` | `FrontierExplorer` node. Subscribes to `/map` (`OccupancyGrid`), detects frontier cells (free cells adjacent to unknown space), and sends `NavigateToPose` goals to `/navigate_to_pose`. |
| `pointtoscan` | `pointcloudtolaserscan.py` | Subscribes to `/rgbd_camera_1/points` (`PointCloud2`), filters points by height (`0.0` to `0.3` m), and publishes `/camera_scan` (`LaserScan`). |
| `pointtofilter` | `pointtofilter.py` | `HeightFilter` node. Subscribes to `/depth_camera_1/points` (`PointCloud2`), filters points with `0.02 < z < 0.20` m, and publishes `/filtered_points`. |
| `flitertoscan` | `flitertoscan.py` | Subscribes to `/filtered_points` (`PointCloud2`) and publishes `/camera_scan` (`LaserScan`). |
| `cam_scan` | `pointtoscan.py` | Subscribes to `/depth_camera_1/points` (`PointCloud2`, 60° HFOV downward-facing camera, `z` in `[-2.0, -0.05]`) and publishes `/scan` (`LaserScan`). |

#### Configuration Files (`config/`)
- `slam_toolbox_params.yaml` — SLAM Toolbox mapping parameters (`base_frame: base_footprint`, `odom_frame: odom`, `map_frame: map`, `scan_topic: /scan`).
- `navigation_2.yaml` — Active Nav2 parameter file used by `nav2_launch.py` (configured with `nav2_mppi_controller::MPPIController` and `bt_navigator`).
- Additional reference configs: `NAV2_PARAM.yaml`, `nav2.yaml`, `nav2_git.yaml`, `navigation_1.yaml`.

### 3. `aws_robomaker_hospital_world`
Provides the Gazebo hospital simulation worlds and 3D assets:
- **Worlds** (`worlds/`): `hospital.world`, `hospital_two_floors.world`, `hospital_three_floors.world`, `myworld.world`
- **Models & Assets**: Hospital furniture, rooms, structural walls, and `fuel_utility.py`.

---

## Prerequisites

- **ROS 2 Jazzy**
- **Gazebo Sim** (`gz sim`) with `ros_gz_sim` and `ros_gz_bridge`
- **ROS 2 Packages**:
  - `robot_state_publisher`, `joint_state_publisher`, `xacro`, `rviz2`
  - `slam_toolbox`
  - `nav2_bringup`, `nav2_msgs`
  - `rtabmap_launch`
  - `sensor_msgs_py`, `python3-numpy`

---

## Build Instructions

```bash
cd /path/to/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note on Hardcoded Paths:**  
> `src/agv_description/launch/display.launch.py` and `src/autonomous_navigation/launch/display.launch.py` currently reference `~/Desktop/Balasubramaniyan/ros2_ws/src/...` for `robot_path` and `world_path`. Update these paths to match your workspace location before launching if your workspace is stored elsewhere.

---

## Usage

Source the workspace in each terminal before running commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. Start Gazebo Simulation & RViz2
```bash
ros2 launch autonomous_navigation display.launch.py
```
This starts Gazebo with `myworld.world`, spawns the robot, bridges topics (`/cmd_vel`, `/scan`, `/odom`, `/tf`, `/clock`, `/joint_states`, `/camera/*`, `/depth_camera_1/points`), and opens RViz2.

### 2. Run SLAM (2D Mapping)
```bash
ros2 launch autonomous_navigation slam.launch.py
```

*(Optional)* For 3D RGB-D mapping with RTAB-Map instead:
```bash
ros2 launch autonomous_navigation rtabmap.launch.py
```

### 3. Start Nav2 Navigation Stack
```bash
ros2 launch autonomous_navigation nav2_launch.py
```

### 4. Run Autonomous Frontier Exploration
```bash
ros2 run autonomous_navigation explore
```

### 5. Optional PointCloud-to-LaserScan Nodes
Convert depth camera point clouds to 2D laser scans for obstacle avoidance or mapping:
```bash
# Direct depth camera (/depth_camera_1/points) -> /scan
ros2 run autonomous_navigation cam_scan

# Or two-stage height-filtered pipeline (/depth_camera_1/points -> /filtered_points -> /camera_scan)
ros2 run autonomous_navigation pointtofilter
ros2 run autonomous_navigation flitertoscan
```

---

## Bridged Topics (`autonomous_navigation/launch/display.launch.py`)

| Topic | ROS 2 Message Type | Direction |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 ↔ Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS 2 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS 2 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS 2 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS 2 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS 2 |
| `/camera/image` | `sensor_msgs/msg/Image` | Gazebo → ROS 2 |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Gazebo → ROS 2 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS 2 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS 2 |
| `/depth_camera_1/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS 2 |

