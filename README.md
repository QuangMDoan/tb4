# TurtleBot 4 Semantic Perception Navigation

**Learning-based perception + geometric depth fusion + costmap-driven planning
on a TurtleBot 4 with OAK-D Pro and RPLidar.**

This ROS 2 (Jazzy) workspace adds camera-based semantic obstacle detection to
the standard Nav2 LiDAR navigation stack.  The robot can see, classify, and
avoid objects that a 2-D LiDAR alone would miss (e.g. a stop sign hanging
above scan height), while retaining the full LiDAR costmap for geometric
obstacle avoidance.

---

## Package Overview

| Package | Language | Role |
|---------|----------|------|
| **tb4_perception** | Python | YOLOv8 detector node — RGB → Detection2DArray |
| **tb4_perception_integration** | Python | Camera–depth fusion node — Detection2DArray + stereo depth → SemanticObstacleArray |
| **tb4_perception_layer** | C++ | Nav2 costmap plugin + BT condition node — stamps semantic costs & triggers stop-sign replanning |
| **turtlebot4_navigation** | YAML/XML | Modified Nav2 config — costmap layers, MPPI controller, custom behaviour tree |
| **turtlebot4** *(upstream)* | C++ | Robot description, messages, base node |
| **turtlebot4_desktop** *(upstream)* | — | RViz launch & config |

---

## Architecture Diagram

```
┌──────────────────────────── SENSOR LAYER ─────────────────────────────────┐
│                                                                           │
│  OAK-D Pro (depthai_ros_driver)           RPLidar A1             TF/Odom │
│  ┌─────────────────────────────┐    ┌──────────────┐    ┌──────────────┐ │
│  │ /oakd/rgb/preview/image_raw │    │   /scan       │    │  odom → base │ │
│  │ /oakd/stereo/image_raw      │    │  LaserScan    │    │  TF frames   │ │
│  │ /oakd/stereo/camera_info    │    │               │    │              │ │
│  │ /oakd/rgb/preview/cam_info  │    │               │    │              │ │
│  └──────────┬──────────────────┘    └──────┬───────┘    └──────┬───────┘ │
│             │                              │                   │         │
└─────────────┼──────────────────────────────┼───────────────────┼─────────┘
              │                              │                   │
              ▼                              │                   │
┌─────────────────────────────────┐          │                   │
│  tb4_perception                 │          │                   │
│  ┌───────────────────────────┐  │          │                   │
│  │  yolo_detector_node       │  │          │                   │
│  │  (YOLOv8n, 30 Hz cap)    │  │          │                   │
│  │                           │  │          │                   │
│  │  RGB image ──► YOLO ──►  │  │          │                   │
│  │  Detection2DArray         │  │          │                   │
│  │  /detections              │  │          │                   │
│  └─────────────┬─────────────┘  │          │                   │
│                │                │          │                   │
└────────────────┼────────────────┘          │                   │
                 │                           │                   │
                 ▼                           │                   ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  tb4_perception_integration                                              │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  camera_lidar_fusion_node                                          │  │
│  │                                                                    │  │
│  │  1. Cache Detection2DArray; pair with each depth frame             │  │
│  │  2. Back-project depth pixels → 3-D points (camera frame)         │  │
│  │  3. Voxel downsample (0.05 m) + RANSAC floor-plane removal        │  │
│  │  4. Re-project 3-D points → 2-D depth pixels                      │  │
│  │  5. Map YOLO boxes from RGB pixel space → depth pixel space        │  │
│  │     via normalised camera rays (dual K matrices)                   │  │
│  │  6. Match transformed boxes to projected depth points              │  │
│  │  7. 1-D depth-gap clustering → pick nearest compact cluster       │  │
│  │  8. TF-transform cluster centroids camera_frame → odom            │  │
│  │  9. Temporal tracking with EMA position smoothing                  │  │
│  │ 10. Publish confirmed tracks                                       │  │
│  │                                                                    │  │
│  │  /detections + /oakd/stereo/image_raw ──►                          │  │
│  │      SemanticObstacleArray  /semantic_obstacles                     │  │
│  │      MarkerArray           /fusion_debug_markers  (optional)       │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  SemanticObstacleArray message  (tb4_perception_layer/msg)               │
│                                                                          │
│    header                                                                │
│    obstacles[]                                                           │
│      ├── class_id    (string)   e.g. "person", "stop sign"              │
│      ├── confidence  (float64)                                           │
│      ├── x, y        (float64)  position in odom frame                  │
│      ├── radius      (float64)  class-specific footprint                │
│      ├── cost        (float64)  class-specific costmap cost             │
│      └── track_id    (int32)    persistent tracker ID                   │
│                                                                          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
          ┌────────────────────┼──────────────────────┐
          │                    │                      │
          ▼                    ▼                      ▼
┌─────────────────────── NAV2 COSTMAP LAYERS ──────────────────────────────┐
│                                                                          │
│  Local Costmap (3 × 3 m, 5 Hz, odom frame)                              │
│  Global Costmap (map frame)                                              │
│                                                                          │
│  ┌──────────────────────┐                                                │
│  │  Static Layer        │  (global only — pre-built SLAM map)           │
│  └──────────────────────┘                                                │
│  ┌──────────────────────┐                                                │
│  │  Obstacle Layer      │  LiDAR /scan — marking + clearing             │
│  └──────────────────────┘                                                │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │  Perception Layer  (tb4_perception_layer::PerceptionLayer)       │    │
│  │  custom Nav2 costmap plugin  (pluginlib)                         │    │
│  │                                                                  │    │
│  │  • Subscribes to /semantic_obstacles                             │    │
│  │  • Stamps circular obstacle footprints with class-aware cost     │    │
│  │  • Per-class radius & cost lookup  (person 254, dog 200, …)     │    │
│  │  • Temporal decay via obstacle_timeout (3 s)                     │    │
│  │  • Only writes cost if > existing cell value (max-cost merge)    │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│  ┌──────────────────────┐                                                │
│  │  Inflation Layer     │  safety buffer (radius 0.25 m, factor 5.0)    │
│  └──────────────────────┘                                                │
│                                                                          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────── PLANNING & CONTROL ───────────────────────────────┐
│                                                                          │
│  Global Planner     NavfnPlanner (A*)                                    │
│  Local Controller   MPPI Controller (DiffDrive, 20 Hz)                   │
│  Smoother           SimpleSmoother                                       │
│  Collision Monitor  FootprintApproach on /scan                           │
│  Velocity Smoother  open-loop, 20 Hz                                     │
│                                                                          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌─────────────── BEHAVIOUR TREE (navigate_to_pose_stop_sign.xml) ─────────┐
│                                                                          │
│  Custom BT extending Nav2 navigate_to_pose_w_replanning_and_recovery:    │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐   │
│  │  IsStopSignNearby  (BT::ConditionNode)                            │   │
│  │  tb4_perception_layer::IsStopSignNearby                           │   │
│  │                                                                   │   │
│  │  • Subscribes to /semantic_obstacles                              │   │
│  │  • Looks up robot pose via TF (map → base_link)                   │   │
│  │  • Returns SUCCESS if any "stop sign" obstacle is within          │   │
│  │    distance_threshold (2.0 m) of the robot                        │   │
│  │  • Wrapped in <Inverter> inside a <ReactiveSequence> so that      │   │
│  │    detection forces an immediate global replan around the sign     │   │
│  └───────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  Recovery: ClearCostmap → Spin → Wait → BackUp (round-robin)            │
│                                                                          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────── CONTROL EXECUTION ────────────────────────────────┐
│                                                                          │
│  /cmd_vel  ──►  TurtleBot 4 (Create 3 base)                             │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## ROS 2 Topic Graph

```
/oakd/rgb/preview/image_raw  ──►  yolo_detector_node  ──►  /detections
                                                                 │
/oakd/stereo/image_raw       ──►  camera_lidar_fusion_node  ◄───┘
/oakd/stereo/camera_info     ──►       │
/oakd/rgb/preview/camera_info──►       │
TF (camera → odom)           ──►       │
                                       ├──►  /semantic_obstacles
                                       └──►  /fusion_debug_markers

/semantic_obstacles  ──►  PerceptionLayer (local_costmap)
/semantic_obstacles  ──►  PerceptionLayer (global_costmap)
/semantic_obstacles  ──►  IsStopSignNearby (BT condition)

/scan                ──►  ObstacleLayer (both costmaps)
/scan                ──►  CollisionMonitor
```

---

## Custom Messages

### SemanticObstacle.msg

```
std_msgs/Header header
string   class_id        # YOLO class name ("person", "stop sign", …)
float64  confidence      # detection confidence [0, 1]
float64  x               # position in odom frame (metres)
float64  y
float64  radius          # obstacle footprint radius
float64  cost            # costmap cost [0, 254]
int32    track_id        # persistent tracker ID
```

### SemanticObstacleArray.msg

```
std_msgs/Header header
tb4_perception_layer/SemanticObstacle[] obstacles
```

---

## Class-Specific Parameters

| Class | Radius (m) | Costmap Cost | Notes |
|-------|-----------|-------------|-------|
| person | 0.30 | 254 (lethal) | Full avoidance |
| chair | 0.25 | 254 (lethal) | Full avoidance |
| bicycle | 0.40 | 254 (lethal) | Wide footprint |
| stop sign | 0.30 | 254 (lethal) | Also triggers BT replan |
| dog | 0.20 | 200 | High cost, not strictly lethal |
| cat | 0.15 | 180 | Moderate cost |

---

## Key Design Decisions

1. **No time-synchronisation between RGB and depth.**  
   The OAK-D RGB and stereo streams use different timestamp bases, so
   `ApproximateTimeSynchronizer` cannot match them reliably. Instead the
   fusion node caches the latest `Detection2DArray` and pairs it with each
   incoming depth frame.

2. **Dual-intrinsics box remapping.**  
   YOLO runs on the 250×250 RGB preview while depth arrives at 640×480.
   Bounding-box corners are converted from detection pixel space to depth
   pixel space via normalised camera rays using both K matrices, avoiding
   any image resize.

3. **Voxel downsample before RANSAC.**  
   A 0.05 m voxel grid typically reduces the point count by ~10× on a
   480×300 depth image, making RANSAC floor removal and all downstream
   steps significantly faster.

4. **1-D depth-gap clustering.**  
   Within each bounding box, depth values are sorted and split where
   consecutive gaps exceed a threshold (0.3 m). The nearest compact
   cluster is selected — this rejects background points that fall inside
   the 2-D box but belong to a different depth layer.

5. **EMA temporal tracking.**  
   Each detection is associated to existing tracks by class + Euclidean
   distance (< 0.5 m). Position is smoothed with exponential moving
   average (α = 0.3). Tracks require `min_hits` (3) before being published
   and are deleted after `max_misses` (5) or `track_timeout` (1.0 s).

6. **Costmap max-cost merge.**  
   `PerceptionLayer::updateCosts` only writes a cell cost when the
   semantic cost exceeds the existing value, so LiDAR lethal cells are
   never downgraded.

7. **BT stop-sign replan.**  
   The custom behaviour tree wraps `ComputePathToPose` in a
   `ReactiveSequence` with an `Inverter(IsStopSignNearby)` condition.
   When a stop sign appears within 2 m the sequence re-ticks the planner,
   forcing an immediate global replan that routes around the now-lethal
   costmap stamp.

---

## Build & Launch

```bash
# Build all custom packages
cd ~/turtlebot4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb4_perception_layer
source install/setup.bash
colcon build --packages-select tb4_perception tb4_perception_integration turtlebot4_navigation

# Terminal 1 — Localisation
ros2 launch turtlebot4_navigation localization.launch.py map:=<map.yaml>

# Terminal 2 — YOLO detector
ros2 launch tb4_perception yolo_detector.launch.py

# Terminal 3 — Camera–depth fusion  (start before Nav2)
ros2 launch tb4_perception_integration perception_integration.launch.py

# Terminal 4 — RViz
ros2 launch turtlebot4_viz view_robot.launch.py

# Terminal 5 — Nav2  (start last)
ros2 launch turtlebot4_navigation nav2.launch.py
```

> **Note:** Launch the fusion node *before* Nav2 so that DDS discovery
> settles before navigation starts.

---

## Debugging

| Alias | Equivalent |
|-------|-----------|
| `tb4 yolo viz` | YOLO detector with `publish_visualisation:=true` |
| `tb4 fusion viz` | Fusion node with `publish_debug_markers:=true` |
| `tb4 bbox_rate` | `ros2 topic hz /detections` |
| `tb4 depth_cam_rate` | `ros2 topic hz /oakd/stereo/image_raw` |
| `tb4 obstacles` | `ros2 topic hz /semantic_obstacles` |

```bash
# Inspect semantic obstacles
ros2 topic echo /semantic_obstacles

# Check costmap layer is active
ros2 param get /local_costmap/local_costmap perception_layer.enabled
```

---

## Workspace Structure

```
src/
├── tb4_perception/                   # YOLO detection (ament_python)
│   ├── tb4_perception/
│   │   └── yolo_detector_node.py     # YOLOv8 inference node
│   ├── config/
│   │   ├── yolo_detector.yaml        # model path, classes, thresholds
│   │   └── oakd_pro.yaml             # OAK-D driver config
│   └── launch/
│       ├── yolo_detector.launch.py
│       └── oakd.launch.py
│
├── tb4_perception_integration/       # Depth fusion (ament_python)
│   ├── tb4_perception_integration/
│   │   └── camera_lidar_fusion_node.py
│   ├── config/
│   │   └── fusion_params.yaml        # RANSAC, clustering, tracking params
│   └── launch/
│       └── perception_integration.launch.py
│
├── tb4_perception_layer/             # Costmap plugin + BT node (ament_cmake)
│   ├── msg/
│   │   ├── SemanticObstacle.msg
│   │   └── SemanticObstacleArray.msg
│   ├── include/tb4_perception_layer/
│   │   ├── perception_layer.hpp
│   │   └── is_stop_sign_nearby.hpp
│   ├── src/
│   │   ├── perception_layer.cpp      # Nav2 costmap layer plugin
│   │   └── is_stop_sign_nearby.cpp   # BT condition node
│   └── plugins/
│       └── perception_layer_plugin.xml
│
├── turtlebot4/                       # Upstream (modified nav config)
│   └── turtlebot4_navigation/
│       └── config/
│           ├── nav2.yaml             # costmap layers + MPPI tuning
│           └── navigate_to_pose_stop_sign.xml  # custom BT
│
└── turtlebot4_desktop/               # Upstream (RViz)
```

---

## License

Apache-2.0