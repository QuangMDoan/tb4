# TurtleBot4 Navigation Config Analysis
Date: 2026-03-25
Files analyzed: slam.yaml, localization.yaml, nav2.yaml, map_101.yaml, map_101.pgm

---

## localization.yaml — AMCL Changes

| Parameter | Was | Now | Why |
|---|---|---|---|
| `laser_max_range` | `100.0` | `12.0` | **Bug.** RPLidar on TurtleBot4 has ~12 m range. Using 100 m completely corrupts the likelihood-field sensor model by expecting obstacles at impossible distances. |
| `laser_min_range` | `-1.0` | `0.15` | Negative range is invalid. 0.15 m matches the RPLidar's minimum range and the collision monitor's `min_height`. |
| `max_beams` | `60` | `120` | More beams → more laser rays used per update → more accurate pose estimate. |
| `max_particles` | `2000` | `5000` | More particles = faster and more confident convergence, especially at startup or after being pushed off pose. |
| `sigma_hit` | `0.2` | `0.1` | RPLidar accuracy is ~2–5 cm. A 20 cm standard deviation is far too loose; 10 cm is a better fit. |
| `recovery_alpha_fast` | `0.0` | `0.1` | With both recovery alphas at 0, AMCL cannot inject new particles when it diverges ("kidnapped robot" problem). Non-zero values trigger random particle injection when the filter's short-term average weight diverges from the long-term average. |
| `recovery_alpha_slow` | `0.0` | `0.001` | See above. |
| `transform_tolerance` | `1.0` | `0.5` | A 1-second TF age tolerance is too relaxed and can mask latency issues. 0.5 s is standard. |
| `update_min_d` | `0.25` | `0.10` | Updating only every 25 cm means the filter lags significantly during slow/precise movements. 10 cm keeps the pose estimate fresh. |
| `z_hit` | `0.5` | `0.7` | Corrects sensor model weights (see z_rand note below). |
| `z_rand` | `0.5` | `0.2` | **Bug.** Modeling 50% of all laser returns as random noise is far too pessimistic and degrades localization severely. The four z-weights must sum to 1.0: 0.7 + 0.05 + 0.05 + 0.2 = 1.0 ✓ |

---

## nav2.yaml — Nav2 Stack Changes

### Controller Server

| Parameter | Was | Now | Why |
|---|---|---|---|
| `vx_max` (MPPI) | `0.5` | `0.26` | **Bug.** The velocity_smoother hard-caps to 0.26 m/s. MPPI was sampling and scoring trajectories up to 0.5 m/s — trajectories the robot can never execute. Aligning these makes MPPI rollouts accurate. |
| `vx_min` (MPPI) | `-0.35` | `-0.26` | Same reason — symmetric with the velocity smoother's reverse limit. |
| `xy_goal_tolerance` | `0.05` | `0.10` | 5 cm is difficult to reliably achieve given localization uncertainty. 10 cm is standard for reliable waypoint arrival. |
| `yaw_goal_tolerance` | `0.05` | `0.10` | ~3° tolerance was barely achievable in practice. ~5.7° is reliable while still being accurate. |

### Local Costmap

| Parameter | Was | Now | Why |
|---|---|---|---|
| `resolution` | `0.01` | `0.05` | The 3×3 m local costmap at 1 cm resolution = 90,000 cells updated at 5 Hz. At 5 cm = 3,600 cells. Saves ~25× CPU on the Raspberry Pi 4 with no meaningful loss in obstacle avoidance. |
| `inflation_radius` | `0.05` | `0.25` | **Critical bug.** The robot circumradius is 0.189 m. At 5 cm inflation, the robot drives directly into walls. 0.25 m = robot radius (0.189 m) + ~0.06 m safety buffer for localization error and map uncertainty. |
| `cost_scaling_factor` | `4.0` | `3.5` | Slightly softer cost falloff over the wider inflation zone to avoid over-constraining paths in tighter spaces. |

### Global Costmap

| Parameter | Was | Now | Why |
|---|---|---|---|
| `resolution` | `0.01` | `0.05` | At 1 cm, a large map = millions of cells updated every second. At 5 cm = ~40,000 cells. A* path planning is orders of magnitude faster. |
| `inflation_radius` | `0.05` | `0.55` | Same as local costmap — critical bug, see above. |
| `cost_scaling_factor` | `4.0` | `3.5` | Same as local costmap. |

### Planner Server

| Parameter | Was | Now | Why |
|---|---|---|---|
| `use_astar` | `false` | `true` | A* is faster than Dijkstra for goal-directed planning with no quality loss. |

---

## slam.yaml — No Changes Required

- `resolution: 0.01` correctly matches `map_101.yaml` (resolution: 0.010). Consistent.
- `mode: mapping` is correct for the mapping phase.
- `max_laser_range: 12.0` correctly matches the RPLidar's range.
- Loop closure and scan matching parameters are reasonable for indoor environments.

**Important workflow note:** During a navigation mission (localization + waypoint following),
do NOT run slam_toolbox. Instead run `map_server` + `amcl` using the tuned `localization.yaml`.
slam_toolbox is only used during the initial mapping phase.
If you prefer to use slam_toolbox's built-in localization mode instead of AMCL,
change `mode: mapping` to `mode: localization` in slam.yaml.

---

## map_101.yaml — Consistency Check

```
resolution: 0.010   ← matches slam.yaml resolution: 0.01 ✓
origin: [-3.417, -6.477, 0]
occupied_thresh: 0.65  ← matches localization.yaml map_saver occupied_thresh_default: 0.65 ✓
free_thresh: 0.196     ← close to map_saver free_thresh_default: 0.25 (acceptable, saved value wins)
```

No issues. The saved map is consistent with the config that generated it.

---

## Summary of Issues (3/25)

1. **inflation_radius: 0.05** — Robot would collide with every obstacle; the costmap provided zero practical clearance.
2. **laser_max_range: 100.0 in AMCL** — Sensor model expects returns at impossible ranges, corrupting particle weights and causing AMCL to fail to converge.
3. **z_rand: 0.5 in AMCL** — Half of all laser readings treated as noise; the z-weights also summed to 1.1 instead of 1.0, an invalid probability distribution.
4. **vx_max: 0.5 vs velocity_smoother cap of 0.26** — MPPI planned trajectories the robot physically cannot follow, causing tracking errors and goal failures.
