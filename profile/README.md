## Project Overview

This is the **iCrane** ROS2 workspace - an industrial robotics system for crane operations with multi-sensor computer vision capabilities. The system integrates MultiSense stereo cameras, Livox/Velodyne LiDAR, and bullet cameras to perform object detection, tracking, and 3D localization.

## Workspace Structure

```
icrane_ws/
└── src/
    ├── icrane_msgs/            # Custom ROS2 message definitions
    ├── icrane_vision/          # Computer vision nodes
    ├── icrane_bringup/         # Launch files and configuration
    ├── multisense_ros2/        # MultiSense camera driver (metapackage)
    │   ├── multisense_lib/     # Hardware communication library
    │   ├── multisense_msgs/    # MultiSense custom messages
    │   └── multisense_ros/     # Main MultiSense driver node
    ├── livox_ros_driver2/      # Livox LiDAR ROS2 driver
    └── Livox-SDK2/             # Livox LiDAR SDK (dependency, not a ROS2 package)
```

**Critical**: This workspace follows ROS2 conventions. The working directory `/icrane_ws/src` contains package source code, but build commands must be run from `/icrane_ws`.

## Build & Development Commands

### Building the Workspace

```bash
# From workspace root (/icrane_ws)
cd /icrane_ws
colcon build
```

Build specific package:
```bash
# Core iCrane packages
colcon build --packages-select icrane_msgs
colcon build --packages-select icrane_vision
colcon build --packages-select icrane_bringup

# Hardware driver packages
colcon build --packages-select multisense_lib multisense_msgs multisense_ros
colcon build --packages-select livox_ros_driver2
```

Build with symlink install (for Python-only packages during development):
```bash
colcon build --symlink-install
```

### Sourcing the Workspace

After building, source the workspace:
```bash
source install/setup.bash
```

### Running Nodes

Launch files (recommended for multi-node systems):
```bash
ros2 launch icrane_bringup icrane_recognition.launch.py
ros2 launch icrane_bringup icrane_object_overlay.launch.py
ros2 launch icrane_bringup icrane_interface.launch.py
```

Individual nodes:
```bash
ros2 run icrane_vision object_detection_general camera_name:=$CAMERA_NAME
ros2 run icrane_vision image_2_world camera_name:=$CAMERA_NAME
ros2 run icrane_vision object_overlay
```

## Architecture & Data Flow

### Package Dependencies

```
Hardware Layer:
    multisense_lib → multisense_msgs → multisense_ros (MultiSense camera driver)
    Livox-SDK2 → livox_ros_driver2 (Livox LiDAR driver)

Application Layer:
    icrane_msgs (interface layer - custom message definitions)
        ↓ (provides custom messages)
    icrane_vision (processing layer - computer vision nodes)
        ↓ (uses launch configurations)
    icrane_bringup (orchestration layer - launch files)
```

**Build order**:
1. **Hardware drivers**: multisense_msgs and livox_ros_driver2 (if using sensors)
2. **icrane_msgs** must be built before icrane_vision (provides custom message definitions)
3. **icrane_vision** must be built before icrane_bringup (provides executable nodes)

### Vision Pipeline Architecture

The system implements a **multi-stage computer vision pipeline**:

```
1. Detection Stage:
   Camera/LiDAR → object_detection_general → DetectionSeg2DArray (2D detections)

2. 3D Projection Stage:
   DetectionSeg2DArray + Depth + CameraInfo → image_2_world → DetectionSeg3DArray

3. Sensor Fusion Stage:
   DetectionSeg3DArray (cam1) + DetectionSeg3DArray (cam2) → object_overlay → MarkerArray
```

### Critical Nodes

**object_detection_general** (`icrane_vision/icrane_vision/object_detection_general.py`)
- Performs YOLO-based 2D object detection
- Subscribes: `/{camera_name}/aux/image_rect_color` (Image)
- Publishes: `/{camera_name}/detections` (DetectionSeg2DArray)
- Uses hardcoded model path: `/home/icranevs/icrane_ws/src/bullet_cam_node/aquatec-dataset-full-n.pt`
- **NOTE**: Model path is environment-specific and needs to be updated for new deployments

**image_2_world** (`icrane_vision/icrane_vision/image_2_world.py:50`)
- Converts 2D detections to 3D world coordinates using depth information
- Uses `message_filters.ApproximateTimeSynchronizer` to sync 3 topics:
  - Depth images: `/{camera_name}/left/openni_depth`
  - Detections: `/{camera_name}/detections`
  - Camera info: `/{camera_name}/aux/image_rect_color/camera_info`
- Publishes: `{camera_name}/detection3d` (DetectionSeg3DArray)
- **Critical**: Requires proper timestamp synchronization (slop=1.0s)

**object_overlay** (`icrane_vision/icrane_vision/object_overlay.py:15`)
- Multi-camera fusion with duplicate removal using 3D IoU
- **Key concept**: Transforms all objects to common reference frame (`target_frame` parameter) before IoU calculation
- Subscribes to two camera streams: `/multisense_1/detection3d` and `/multisense_2/detection3d`
- Uses TF2 for frame transformations (critical dependency)
- Publishes: `/debug/object_overlay_filtered` (MarkerArray), `/vision_bridge` (VisionBridgeArray)
- Parameters:
  - `iou_threshold`: Default 0.01 (configured in `config/object_overlay_params.yaml`)
  - `target_frame`: Default 'multisense_base_link'

### Message Synchronization

The system heavily relies on **message_filters.ApproximateTimeSynchronizer** for sensor fusion:
- Time synchronization slop varies by node (0.5s to 1.0s)
- All sensor messages must have valid timestamps
- TF tree must be published continuously for frame transformations

## Hardware Driver Packages

### MultiSense Camera Driver (multisense_ros2)

The **multisense_ros2** metapackage provides ROS2 support for Carnegie Robotics MultiSense stereo cameras.

**Package structure**:
- **multisense_lib**: Low-level hardware communication library (wraps LibMultiSense SDK as submodule)
- **multisense_msgs**: Custom ROS2 messages for MultiSense-specific data
- **multisense_ros**: Main driver node that publishes camera images, depth, point clouds, and TF transforms

**Key topics published** (for camera named `multisense_1`):
- `/multisense_1/aux/image_rect_color` - Rectified color image (used by object_detection_general)
- `/multisense_1/left/openni_depth` - Depth image (used by image_2_world)
- `/multisense_1/aux/image_rect_color/camera_info` - Camera calibration (used by image_2_world)
- `/multisense_1/organized_point_cloud` - 3D point cloud

**Launch the driver**:
```bash
ros2 launch multisense_ros multisense.launch.py namespace:=$CAMERA_NAME ip_address:=$CAMERA_IP_ADDRESS

# MultiSense_1 ip address: 10.66.171.21
# MultiSense_2 ip address: 11.66.171.22
```

**Requirements**:
- ROS2 Foxy or later
- Requires `xacro` and `tf2_geometry_msgs` packages
- Supports firmware versions 3.3+
- Does not support monocular or BCAM configurations

**Reference**: https://github.com/carnegierobotics/multisense_ros2

### Livox LiDAR Driver (livox_ros_driver2)

The **livox_ros_driver2** package provides ROS2 support for Livox 3D LiDAR sensors (HAP, Mid-360, etc.).

**Supported sensors**:
- Livox HAP (TX/T1)
- Livox Mid-360
- Other Livox SDK2-compatible lidars

**Key topics published**:
- `/livox/lidar` - Point cloud data (sensor_msgs/PointCloud2)
- `/livox/imu` - IMU data (if available on sensor)

**Dependencies**:
- **Livox-SDK2**: Core SDK library (included as separate package in workspace)
- PCL (Point Cloud Library)
- APR library

**Launch the driver**:
```bash
ros2 launch livox_ros_driver2 livox_lidar_launch.py
```

**Configuration**:
- Config files typically in `livox_ros_driver2/config/`
- Set LiDAR IP addresses, frame IDs, and data types in config

**Reference**:
- Driver: https://github.com/Livox-SDK/livox_ros_driver2
- SDK: https://github.com/Livox-SDK/Livox-SDK2
- Protocols: See Livox-SDK2 wiki for HAP and Mid-360 communication protocols

**Note**: This driver complements or replaces Velodyne LiDAR mentioned in the original system design.

## Configuration Management

### Parameter Files

- `icrane_bringup/config/params.yaml` - YOLO model configuration for object_detection_general
- `icrane_bringup/config/object_overlay_params.yaml` - IoU threshold and target frame for object_overlay

**Current Issue**: `params.yaml` is commented out in `icrane_bringup/setup.py:22`, so parameters are not loaded properly. Nodes use hardcoded defaults.

### Camera Name Convention

Multi-camera support uses the `camera_name` parameter:
- Default cameras: `multisense_1`, `multisense_2`
- Topic naming: `/{camera_name}/{subtopic}`
- Pass via launch args or command line: `--ros-args -p camera_name:=multisense_1`

## Important Implementation Details

### Frame Transformations

The system uses **TF2** extensively for coordinate transformations:
- Primary reference frames: `multisense_base_link`, `base_link`, `map`
- All 3D operations require valid TF tree
- Check TF availability: `ros2 run tf2_tools view_frames`
- Monitor specific transform: `ros2 run tf2_ros tf2_echo [source] [target]`

### YOLO Model Management

Current setup uses YOLOv11-seg models:
- Models are **not** in version control (`.pt` files in `.gitignore` or should be)
- Found model files:
  - `icrane_vision/yolo11n-seg.pt` (nano segmentation model)
  - `icrane_vision/dataset_aquatec_parcial_v4.pt` (custom trained)
- Model paths are **hardcoded** in node source code, not parameterized
- Classes filter: `classes=[0,1]` (person, cardboard-box)
- Confidence threshold: `conf=0.7`

### QoS Profiles

Nodes use `BEST_EFFORT` reliability for sensor data:
```python
qos_profile = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
    depth=1  # or 10 for subscribers
)
```

This is critical for compatibility with hardware sensor drivers.

## Custom Message Types

Key messages defined in **icrane_msgs**:

**Detection Messages**:
- `Detection2D`, `Detection2DArray` - 2D bounding boxes
- `DetectionSeg2D`, `DetectionSeg2DArray` - 2D with segmentation masks
- `DetectionSeg3D`, `DetectionSeg3DArray` - 3D bounding boxes with segmentation

**Integration Messages**:
- `VisionBridge`, `VisionBridgeArray` - Unified detection format for downstream systems
- `TrackedEntities` - Object tracking across frames (planned feature)
- `Prediction` - Trajectory prediction (planned feature)

**Monitoring Messages**:
- `BigBrotherMonitoring`, `BigBrotherAlarms` - System health monitoring
- `AlarmInfo`, `AlarmLimit` - Safety alarm system

**Domain-Specific**:
- `Lance`, `Load` - Crane-specific operations
- `OmniverseInfo` - NVIDIA Omniverse integration (future)

## Known Issues & Workarounds

1. **Commented Configuration**: `icrane_bringup/setup.py:22` has params.yaml commented out. Parameters must be passed via command line or launch files.

2. **Hardcoded Model Paths**: `object_detection_general.py:53` has hardcoded path `/home/icranevs/icrane_ws/...`. Update this for new environments or parameterize it.

3. **Empty Entry Points**: `icrane_bringup` has no executable nodes (`setup.py:31` has empty console_scripts). This package is launch-file only.

4. **Missing Package Descriptions**: All `package.xml` files have "TODO: Package description" and "TODO: License declaration".

5. **TF Timeout Issues**: `object_overlay` uses 1.0s timeout for TF lookups. In systems with irregular TF publishing, this may fail. Consider increasing or adding retry logic.

## Development Workflow

### Adding New Vision Nodes

1. Create node in `icrane_vision/icrane_vision/your_node.py`
2. Add entry point in `icrane_vision/setup.py` console_scripts
3. Create launch file in `icrane_bringup/launch/`
4. Add launch file to `icrane_bringup/setup.py` data_files
5. Build and test:
   ```bash
   cd /Users/pedronunes/icrane_ws
   colcon build --packages-select icrane_vision icrane_bringup
   source install/setup.bash
   ros2 run icrane_vision your_node
   ```

### Adding Custom Messages

1. Define message in `icrane_msgs/msg/YourMessage.msg`
2. Add to `icrane_msgs/CMakeLists.txt` in `rosidl_generate_interfaces()`
3. Rebuild icrane_msgs and dependent packages:
   ```bash
   colcon build --packages-select icrane_msgs
   colcon build --packages-up-to icrane_vision
   ```

### Debugging Vision Pipeline

Use RViz2 for visualization:
```bash
ros2 run rviz2 rviz2
```

Key topics to visualize:
- `/{camera_name}/debug_image_test` - YOLO detection overlays
- `/debug/object_overlay_filtered` - Fused 3D markers
- `/{camera_name}/img_pcd` - Point clouds from detections

Check node connectivity:
```bash
ros2 node list
ros2 node info /object_overlay
ros2 topic hz /multisense_1/detection3d
ros2 topic echo /vision_bridge
```

## External Dependencies

### Core Vision Processing

- **ROS2** (Humble or Iron)
- **OpenCV** 4.x (`python3-opencv`, `cv_bridge`)
- **PyTorch** (for YOLO)
- **Ultralytics** (`ultralytics` Python package)
- **Open3D** (point cloud processing)
- **ros2_numpy** (NumPy integration)

Install Python dependencies:
```bash
pip install ultralytics open3d torch torchvision
```

### Hardware Drivers

**MultiSense (multisense_ros2)**:
```bash
sudo apt install ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-tf2-geometry-msgs
sudo apt install ros-${ROS_DISTRO}-angles ros-${ROS_DISTRO}-image-geometry
```

**Livox LiDAR (livox_ros_driver2)**:
```bash
sudo apt install libpcl-dev libapr1-dev
sudo apt install ros-${ROS_DISTRO}-pcl-conversions
```

**General**:
```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-sensor-msgs
```

## Deployment Notes

### Hardware Requirements

- **MultiSense cameras**: System expects topics `/multisense_*/...` (aux/image_rect_color, left/openni_depth, etc.)
- **LiDAR sensors**:
  - Livox: `/livox/lidar` (point cloud), `/livox/imu` (if available)
  - Velodyne: `/velodyne_points` (legacy support)
- All hardware drivers must publish with compatible QoS profiles (BEST_EFFORT)
- TF tree must include transforms between all sensor frames and base_link

### Software Requirements

- ROS2 Humble or Iron recommended
- Python dependencies: ultralytics, open3d, torch, torchvision
- System packages: xacro, tf2_geometry_msgs
- YOLO model files must be present at configured paths before launching

### Network Configuration

- MultiSense cameras typically require static IP configuration
- Livox LiDAR requires IP address configuration in driver config files
- Ensure network interfaces support jumbo frames for high-bandwidth sensors

