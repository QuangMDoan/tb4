# Nav2 Performance Tuning — Analysis & Recommended Changes

## Environment Context

- **Map**: `map_101.pgm` — 270×249 pixels at 0.05 m/px → ~13.5 m × 12.5 m indoor space
- **Robot**: TurtleBot4 (differential drive, 2D lidar, max 0.26 m/s)
- **Stack**: Nav2 (MPPI controller, NavFn planner, AMCL localization, SLAM Toolbox)

---

## nav2.yaml Changes

### MPPI Controller — Reduced Compute Load

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `time_steps` | 28 | **20** | 28 steps × 0.1s = 2.8s horizon. At 0.26 m/s max speed the robot only covers ~0.73 m in that time. 20 steps (2.0s) is more than sufficient and saves ~30% of MPPI compute. |
| `batch_size` | 1000 | **500** | Halves the number of sampled trajectories per cycle. 500 is plenty for a slow diff-drive robot in a small indoor environment. This is the single biggest CPU saver. |
| `visualize` | true | **false** | Publishing marker arrays at 20 Hz is pure debugging overhead. Disable unless actively tuning. |

### Local Costmap — Switched from VoxelLayer to ObstacleLayer

| Parameter | Old | New | Rationale |
|---|---|---|---|
| Layer plugin | `VoxelLayer` | **`ObstacleLayer`** | VoxelLayer maintains a 3D voxel grid — completely unnecessary with a 2D lidar that only produces a single-plane scan. ObstacleLayer does the same obstacle tracking in 2D at a fraction of the cost. |
| `publish_voxel_map` | true | **(removed)** | No longer applicable. Was publishing a full 3D voxel grid every cycle for no benefit. |

### Both Costmaps — Incremental Updates

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `always_send_full_costmap` | true | **false** | When `true`, the entire costmap grid is serialized and published every update cycle. With `false`, only changed cells are transmitted — dramatically reduces bandwidth and serialization CPU, especially on the global costmap. |

### Smoother — Relaxed Convergence

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `tolerance` | 1.0e-10 | **1.0e-5** | 1e-10 is absurdly tight — the smoother iterates to near-machine-epsilon for imperceptible gains. 1e-5 converges in far fewer iterations with identical practical path quality. |
| `max_its` | 1000 | **500** | Paired with the relaxed tolerance, 500 iterations is more than enough. Prevents worst-case long stalls. |

---

## localization.yaml Changes (AMCL)

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `max_particles` | 5000 | **2000** | The map is only ~13.5 m × 12.5 m. 5000 particles is overkill — most will be redundant in a small, well-featured space. 2000 still provides excellent coverage while cutting particle filter CPU by ~60%. |
| `min_particles` | 500 | **200** | Allows the adaptive particle filter to shrink further when the robot is well-localized, saving cycles during steady-state navigation. |
| `max_beams` | 120 | **60** | Each particle evaluates `max_beams` lidar rays. Halving from 120→60 halves per-particle sensor model cost. 60 beams is still very dense for a small map with clear features. |
| `resample_interval` | 1 | **2** | Resampling every update is expensive and can cause particle depletion. Every 2nd update is the standard recommendation — saves compute and improves particle diversity. |

---

## slam.yaml Changes (SLAM Toolbox)

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `throttle_scans` | 1 | **2** | Process every 2nd scan instead of every scan. At a typical 10 Hz lidar rate, this means 5 Hz scan matching — still fast enough for 0.26 m/s max speed, but halves SLAM compute. |
| `map_update_interval` | 1.0 | **2.0** | Updating the occupancy grid map every 2s instead of 1s. The robot moves at most ~0.5 m between updates, so map freshness is barely affected. |
| `correlation_search_space_resolution` | 0.01 | **0.02** | The correlation search grid at 0.01 m was finer than the map resolution (0.05 m). At 0.02 m, the search grid has 4× fewer cells to evaluate, and is still well below map resolution so matching accuracy is preserved. |
| `loop_search_space_dimension` | 8.0 | **6.0** | An 8 m search radius exceeds half the map dimensions. 6 m still covers any realistic loop closure candidate in this environment while reducing the search space by ~44%. |

---

## What Was Deliberately NOT Changed

These parameters were reviewed and intentionally left alone:

| Parameter | Value | Reason to keep |
|---|---|---|
| `controller_frequency` | 20 Hz | Already well-matched to MPPI. Reducing further would degrade tracking quality. |
| Costmap `resolution` | 0.05 m | Matches the map resolution. Coarsening would lose obstacle detail. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.10 | These define navigation accuracy, not speed. Loosening them trades precision for nothing meaningful — the robot is already slow. |
| `use_astar` | true | A* is already the fastest option for NavFn on a small map. |
| `collision_monitor` | (all params) | Safety-critical system. Must not be weakened for performance. |
| `inflation_radius` | 0.30 m | Already tight for a ~0.19 m radius robot. Further reduction risks collisions. |
| Global costmap `update_frequency` | 1.0 Hz | Already low. The global costmap is less performance-sensitive anyway. |
| `do_loop_closing` | true | Essential for map quality in SLAM. Disabling would save compute but produce drift. |

---

## Expected Impact

- **MPPI controller**: ~50% reduction in per-cycle compute (fewer batches + fewer timesteps)
- **Costmaps**: Significant reduction in CPU and bandwidth from ObstacleLayer switch and incremental updates
- **AMCL**: ~60% reduction in particle filter compute
- **SLAM**: ~50% reduction in scan processing load
- **Accuracy**: Negligible impact — all changes stay well within safe margins for this map size and robot speed
