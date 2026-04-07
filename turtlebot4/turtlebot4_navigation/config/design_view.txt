+==================================================================================================+
|                          AUTONOMOUS NAVIGATION SYSTEM ARCHITECTURE                               |
|            Learning-Based Perception + Geometric Fusion + Costmap-Based Planning                 |
+==================================================================================================+

                ┌──────────────────────────── SENSOR LAYER ─────────────────────────────┐
                │                                                                       │
                │   ┌───────────────┐      ┌────────────────┐      ┌─────────────────┐  │
                │   │   Camera      │      │   LiDAR / RGB-D│      │  Odometry / TF  │  │
                │   │   (RGB image) │      │   PointCloud2  │      │  (odom, base)   │  │
                │   └──────┬────────┘      └────────┬───────┘      └────────┬────────┘  │
                │          │                        │                       │           │
                └──────────┼────────────────────────┼───────────────────────┼───────────┘
                           │                        │                       │
                           ▼                        ▼                       ▼

                ┌────────────────────── PERCEPTION LAYER ──────────────────────┐
                │                                                              │
                │   ┌──────────────────────────────┐                           │
                │   │   Object Detection (YOLO)    │                           │
                │   │  - Bounding boxes (2D)       │                           │
                │   │  - Class labels              │                           │
                │   │  - Confidence scores         │                           │
                │   └──────────────┬───────────────┘                           │
                │                  │                                           │
                │                  ▼                                           │
                │   ┌──────────────────────────────┐                           │
                │   │  Camera–LiDAR Fusion Module  │                           │
                │   │                              │                           │
                │   │  - Sync image + point cloud  │                           │
                │   │  - Transform cloud → camera  │                           │
                │   │  - Project 3D → image plane  │                           │
                │   │  - Select points in boxes    │                           │
                │   │  - Estimate object position  │                           │
                │   └──────────────┬───────────────┘                           │
                │                  │                                           │
                └──────────────────┼───────────────────────────────────────────┘
                                   │
                                   ▼

                ┌────────────────── SEMANTIC REPRESENTATION LAYER ───────────────────┐
                │                                                                    │
                │   ┌────────────────────────────────────────────────────────────┐   │
                │   │ SemanticObstacleArray (ROS 2 message)                      │   │
                │   │                                                            │   │
                │   │  { class, confidence, (x,y), radius, cost } in odom/map    │   │
                │   │                                                            │   │
                │   │  - filters low-confidence detections                       │   │
                │   │  - assigns class-based radius and cost                     │   │
                │   │  - enforces temporal consistency                           │   │
                │   └────────────────────────────┬───────────────────────────────┘   │
                │                                │                                   │
                └────────────────────────────────┼───────────────────────────────────┘
                                                 │
                                                 ▼

+==================================================================================================+
|                                NAVIGATION & MAPPING LAYER (Nav2)                                 |
+==================================================================================================+

                ┌──────────────────────── COSTMAP INTEGRATION ───────────────────────┐
                │                                                                    │
                │   ┌──────────────────────┐                                         │
                │   │  Static Layer        │   (pre-built map / SLAM output)         │
                │   └──────────────────────┘                                         │
                │                                                                    │
                │   ┌──────────────────────┐                                         │
                │   │  Obstacle Layer      │   (LiDAR marking & clearing)            │
                │   └──────────────────────┘                                         │
                │                                                                    │
                │   ┌──────────────────────┐                                         │
                │   │  Perception Layer    │   (custom plugin)                       │
                │   │  MyPerceptionLayer   │                                         │
                │   │                      │                                         │
                │   │  - stamps semantic   │                                         │
                │   │    obstacles         │                                         │
                │   │  - class-aware cost  │                                         │
                │   │  - temporal decay    │                                         │
                │   └──────────────────────┘                                         │
                │                                                                    │
                │   ┌──────────────────────┐                                         │
                │   │  Inflation Layer     │   (safety buffer around obstacles)      │
                │   └──────────────────────┘                                         │
                │                                                                    │
                └───────────────┬────────────────────────────────────────────────────┘
                                │
                                ▼

                ┌──────────────────────────────────────────────────────────────┐
                │           Unified Costmap (Local / Global)                   │
                │   - combined geometric + semantic environment representation │
                └──────────────────────────────┬───────────────────────────────┘
                                               │
                                               ▼

                ┌────────────────────── PLANNING & CONTROL ──────────────────────┐
                │                                                                │
                │   ┌──────────────────────┐                                     │
                │   │  Global Planner      │   (path generation)                 │
                │   └──────────────────────┘                                     │
                │                                                                │
                │   ┌──────────────────────┐                                     │
                │   │  Local Controller    │   (trajectory tracking)             │
                │   └──────────────────────┘                                     │
                │                                                                │
                └──────────────┬─────────────────────────────────────────────────┘
                               │
                               ▼

                ┌────────────────────── CONTROL EXECUTION ───────────────────────┐
                │                                                                │
                │   ┌──────────────────────┐                                     │
                │   │  Velocity Commands   │  (/cmd_vel)                         │
                │   └──────────────┬───────┘                                     │
                │                  │                                             │
                │                  ▼                                             │
                │   ┌──────────────────────┐                                     │
                │   │  TurtleBot 4         │                                     │
                │   │  Motion Execution    │                                     │
                │   └──────────────────────┘                                     │
                │                                                                │
                └────────────────────────────────────────────────────────────────┘

+==================================================================================================+
|                                DEBUGGING & EVALUATION (RViz)                                     |
|                                                                                                  |
| - camera detections (2D)                                                                         |
| - projected 3D points                                                                            |
| - selected object centroids                                                                      |
| - semantic obstacles (odom frame)                                                                |
| - layered costmaps                                                                               |
| - planned path and executed trajectory                                                           |
+==================================================================================================+