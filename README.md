# ieskf_slam

A LiDAR-inertial SLAM system based on the Iterated Error-State Kalman Filter (IESKF), featuring Scan Context loop closure detection, ICP-based geometric verification, and Ceres pose-graph optimization.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Supported Sensors](#supported-sensors)
- [Dependencies](#dependencies)
- [Build](#build)
- [Usage](#usage)
- [Loop Closure Pipeline](#loop-closure-pipeline)
- [Parameter Reference](#parameter-reference)
  - [Front-End Parameters](#front-end-parameters)
  - [Map Parameters](#map-parameters)
  - [IESKF Parameters](#ieskf-parameters)
  - [Back-End: Keyframe Selection](#back-end-keyframe-selection)
  - [Back-End: Scan Context Detection](#back-end-scan-context-detection)
  - [Back-End: Loop Candidate Filtering](#back-end-loop-candidate-filtering)
  - [Back-End: ICP Registration](#back-end-icp-registration)
  - [Back-End: Consistency Checks](#back-end-consistency-checks)
  - [Back-End: Pose Graph Optimization](#back-end-pose-graph-optimization)
  - [Back-End: Map Output](#back-end-map-output)
- [Loop Closure Tuning Guide](#loop-closure-tuning-guide)
  - [Understanding False Positives vs False Negatives](#understanding-false-positives-vs-false-negatives)
  - [Parameter Interaction Map](#parameter-interaction-map)
  - [Tuning Strategy by Scenario](#tuning-strategy-by-scenario)
  - [Recommended Configurations](#recommended-configurations)
  - [Diagnostic Workflow](#diagnostic-workflow)
- [Changelog](#changelog)

---

## Overview

ieskf_slam provides a tightly coupled LiDAR-inertial odometry front-end using IESKF, with a modular back-end that performs:

1. **Keyframe management** with adaptive selection based on translation, rotation, or frame-count thresholds.
2. **Loop closure detection** using Scan Context global descriptors.
3. **Geometric verification** via ICP alignment against local submaps.
4. **Multi-stage filtering** including height-difference rejection, yaw consistency checks, and odometry-consistency gating.
5. **Pose-graph optimization** using Ceres Solver with Levenberg-Marquardt.

## Architecture

```
LiDAR + IMU
    |
    v
[Front-End: IESKF Odometry]
    |
    v  (CloudWithPose)
[Back-End Coordinator]
    |
    +---> Keyframe Selection
    |         |
    |         v
    +---> Scan Context Loop Detection
    |         |
    |         v  (LoopCandidate)
    +---> Height-Difference Filter        <-- commit 877fd168
    |         |
    |         v
    +---> Submap Construction
    |         |
    |         v
    +---> ICP Registration + Consistency Checks  <-- commit 862a84e0
    |         |
    |         v  (LoopConstraint)
    +---> Ceres Pose-Graph Optimization   <-- commit 862a84e0
              |
              v
         Optimized Trajectory + Map
```

## Supported Sensors

| Sensor Type | Config File | `lidar_type` |
|---|---|---|
| Livox Avia | `config/avia.yaml` | 0 |
| Velodyne (VLP-16/HDL-32/HDL-64) | `config/velodyne.yaml` | 1 |
| Generic PointCloud2 | `config/generic.yaml` | 2 |

## Dependencies

- ROS Noetic (Ubuntu 20.04)
- PCL >= 1.10
- Eigen >= 3.3
- Ceres Solver >= 2.0
- nanoflann (bundled)

## Build

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ssg135/ieskf_slam.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Usage

```bash
# Livox Avia
roslaunch ieskf_slam avia_backend.launch

# Velodyne
roslaunch ieskf_slam velodyne_backend.launch

# Play a rosbag
rosbag play your_dataset.bag
```

---

## Loop Closure Pipeline

The back-end processes each incoming frame through a multi-stage pipeline. Understanding this pipeline is essential for parameter tuning.

### Stage 1: Keyframe Selection

A new keyframe is created when **any** of the following conditions is met:
- Translation from the last keyframe exceeds `keyframe_translation_thresh` (default: 1.0 m)
- Rotation from the last keyframe exceeds `keyframe_rotation_thresh_deg` (default: 10.0 deg)
- Frame count since the last keyframe reaches `force_keyframe_every_n` (default: 30)

### Stage 2: Scan Context Detection

Each keyframe's downsampled cloud is converted to a Scan Context descriptor (20 rings x 60 sectors, max height encoding). The descriptor is compared against a KD-tree of historical ring-keys, excluding the most recent 30 keyframes (`NUM_EXCLUDE_RECENT`). The top 3 nearest neighbors (`NUM_CANDIDATES_FROM_TREE`) are evaluated pairwise, and the best match below `scan_context_distance_threshold` becomes a loop candidate.

**Key insight**: The SC distance is `1 - cosine_similarity`, ranging from 0 (identical) to 2 (opposite). The default threshold of 0.15 means requiring cosine similarity >= 0.85.

### Stage 3: Height-Difference Filter (commit 877fd168)

Before ICP, the candidate is rejected if the Z-axis (height) difference between the current keyframe and the target keyframe exceeds `loop_candidate_max_height_diff_m`. This prevents false matches between different floors or elevation levels.

### Stage 4: Submap Construction

A local submap is built around the target keyframe by aggregating `loop_submap_num_keyframes_each_side` neighboring keyframes (default: 10 on each side = 21 total), transformed into the target frame and downsampled with `loop_submap_voxel_leaf_size`.

### Stage 5: ICP Registration

Point-to-point ICP aligns the current keyframe against the target submap. The initial transform comes from odometry (relative optimized poses). The registration is validated through three gates:

1. **Yaw consistency**: The yaw angle from Scan Context must agree with odometry-derived yaw within `loop_candidate_max_yaw_diff_deg_from_odom`.
2. **Convergence + fitness**: ICP must converge and the mean squared correspondence distance must be below `icp_fitness_threshold`.
3. **Odometry consistency**: The ICP-refined pose must not deviate from the initial odometry guess by more than `loop_max_translation_delta_from_guess` (translation) or `loop_max_rotation_delta_deg_from_guess` (rotation).

### Stage 6: Pose-Graph Optimization

Accepted loop constraints are added as edges to the pose graph alongside odometry edges. Ceres Solver (Levenberg-Marquardt) optimizes all poses. If optimization fails, the loop edge is removed (rollback).

---

## Parameter Reference

All parameters are set in the YAML config files under `config/`. Parameters are loaded under the `back_end/` namespace.

### Front-End Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `front_end/extrin_r` | double[9] | Identity | Rotation matrix (row-major) from LiDAR to IMU frame |
| `front_end/extrin_t` | double[3] | [0,0,0] | Translation vector from LiDAR to IMU frame |

### Map Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `map/type` | string | `ikdtree` | Map structure type (`ikdtree` or `rect_kdtree`) |
| `map/map_side_length_2` | double | 500 | Half-length of the map bounding box (m) |
| `map/map_resolution` | double | 0.5 | Map voxel resolution (m) |

### IESKF Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ieskf/cov_gyroscope` | double | 0.1 | Gyroscope noise covariance |
| `ieskf/cov_acceleration` | double | 0.1 | Accelerometer noise covariance |
| `ieskf/cov_bias_gyroscope` | double | 0.0001 | Gyroscope bias random walk |
| `ieskf/cov_bias_acceleration` | double | 0.0001 | Accelerometer bias random walk |

### Back-End: Keyframe Selection

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `keyframe_translation_thresh` | double | 1.0 | m | Min translation to create a new keyframe |
| `keyframe_rotation_thresh_deg` | double | 10.0 | deg | Min rotation to create a new keyframe |
| `force_keyframe_every_n` | int | 30 | frames | Force keyframe after N frames without one |
| `keyframe_voxel_leaf_size` | double | 0.5 | m | Voxel filter leaf size for keyframe clouds |

### Back-End: Scan Context Detection

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `scan_context_distance_threshold` | double | 0.15 | - | Max SC distance to accept a loop candidate. Range [0, 2]; lower = stricter. **Primary gate for false positive control.** |

**Hardcoded Scan Context constants** (in `Scancontext.h`):

| Constant | Value | Description |
|---|---|---|
| `PC_NUM_RING` | 20 | Number of radial rings in the descriptor |
| `PC_NUM_SECTOR` | 60 | Number of angular sectors in the descriptor |
| `PC_MAX_RADIUS` | 80.0 m | Maximum range for descriptor encoding |
| `LIDAR_HEIGHT` | 2.0 m | Assumed LiDAR mounting height (added to z-values) |
| `NUM_EXCLUDE_RECENT` | 30 | Exclude this many recent keyframes from loop search |
| `NUM_CANDIDATES_FROM_TREE` | 3 | Number of KNN candidates to evaluate |
| `TREE_MAKING_PERIOD_` | 10 | Rebuild KD-tree every N keyframes |
| `SEARCH_RATIO` | 0.1 | Column-shift search range (fraction of total sectors) |

### Back-End: Loop Candidate Filtering

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `loop_candidate_max_height_diff_m` | double | 1.5 | m | Max Z-axis height difference between query and target keyframes. Set 0 to disable. **Introduced in commit 877fd168.** |
| `loop_submap_num_keyframes_each_side` | int | 10 | - | Number of keyframes on each side of the target to include in the submap (total = 2N+1) |
| `loop_submap_voxel_leaf_size` | double | 0.5 | m | Voxel filter leaf size for submap downsampling |

### Back-End: ICP Registration

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `icp_max_iterations` | int | 30 | - | Maximum ICP iterations |
| `icp_max_correspondence_distance` | double | 5.0 | m | Maximum point-to-point correspondence distance for ICP |
| `icp_fitness_threshold` | double | 0.4 | m^2 | Maximum mean squared correspondence distance to accept ICP result. **Critical for false positive control.** |

### Back-End: Consistency Checks

These parameters form a second defense layer beyond Scan Context and ICP fitness, rejecting loops where the ICP result is inconsistent with odometry.

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `loop_candidate_max_yaw_diff_deg_from_odom` | double | 35.0 | deg | Max yaw angle disagreement between Scan Context initial estimate and odometry-derived relative yaw. **Introduced in commit 862a84e0.** |
| `loop_max_translation_delta_from_guess` | double | 3.0 | m | Max translation difference between ICP-refined pose and odometry initial guess. **Introduced in commit 862a84e0.** |
| `loop_max_rotation_delta_deg_from_guess` | double | 20.0 | deg | Max rotation difference between ICP-refined pose and odometry initial guess. **Introduced in commit 862a84e0.** |

### Back-End: Pose Graph Optimization

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `odom_translation_information` | double | 100.0 | - | Information matrix diagonal for odometry translation. Higher = more trust in odometry. |
| `odom_rotation_information` | double | 150.0 | - | Information matrix diagonal for odometry rotation |
| `loop_translation_information` | double | 30.0 | - | Base information matrix diagonal for loop translation (scaled by 1/fitness) |
| `loop_rotation_information` | double | 50.0 | - | Base information matrix diagonal for loop rotation (scaled by 1/fitness) |
| `optimizer_max_iterations` | int | 15 | - | Ceres solver max iterations |
| `optimizer_stop_delta_norm` | double | 0.0001 | - | Ceres parameter convergence tolerance |
| `optimizer_damping_lambda` | double | 0.000001 | - | Ceres LM minimum diagonal damping |

**Note**: Loop constraint information is scaled by `1 / max(fitness, 1e-3)`, so better ICP fits produce stronger constraints.

### Back-End: Map Output

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `optimized_map_publish_hz` | double | 1.0 | Hz | Rate limit for optimized map publishing |
| `map_visualization_radius` | double | 30.0 | m | Radius around latest pose for map visualization |
| `map_visualization_min_recent_keyframes` | int | 20 | - | Always include at least this many recent keyframes in visualization |
| `map_visualization_max_keyframes` | int | 80 | - | Cap total keyframes in visualization (0 = unlimited) |
| `map_voxel_leaf_size` | double | 0.5 | m | Voxel filter for visualization map |
| `dense_map_keyframe_voxel_leaf_size` | double | 0.3 | m | Per-keyframe voxel filter for dense map output |
| `dense_map_global_voxel_leaf_size` | double | 0.3 | m | Global voxel filter for dense map output |
| `enable_record_raw` | bool | false | - | Write raw keyframe poses to file |
| `enable_record_optimized` | bool | false | - | Write optimized keyframe poses to file |
| `enable_save_dense_map` | bool | true | - | Save dense PCD map on shutdown |

---

## Loop Closure Tuning Guide

### Understanding False Positives vs False Negatives

| Type | Effect | Symptom |
|---|---|---|
| **False Positive** | Incorrect loop accepted | Map distortion, trajectory jumps, localization divergence after optimization |
| **False Negative** | Correct loop rejected | Accumulated drift not corrected, map misalignment on revisit |

**False positives are far more dangerous** than false negatives. A single false positive can corrupt the entire map, while false negatives only leave uncorrected drift. Always err on the side of conservative thresholds.

### Parameter Interaction Map

The loop closure pipeline applies filters in sequence. Each stage can reject a candidate, and only candidates passing ALL stages become accepted loop constraints.

```
Scan Context Distance        -- Stage gate: is the scene similar?
       |
       v  (pass if distance < scan_context_distance_threshold)
Height Difference Filter     -- Geometric gate: same elevation?
       |
       v  (pass if |dz| < loop_candidate_max_height_diff_m)
Yaw Consistency Check        -- Orientation gate: SC and odom agree on heading?
       |
       v  (pass if yaw_delta < loop_candidate_max_yaw_diff_deg_from_odom)
ICP Convergence + Fitness    -- Alignment gate: do the point clouds align well?
       |
       v  (pass if converged AND fitness < icp_fitness_threshold)
Odometry Consistency Check   -- Sanity gate: ICP result reasonable vs. odom?
       |
       v  (pass if translation_delta < max AND rotation_delta < max)
ACCEPTED LOOP CONSTRAINT
```

**Key insight**: Tightening an early stage saves computation (avoids unnecessary ICP). Tightening a late stage is more precise (uses richer information).

### Tuning Strategy by Scenario

#### Scenario 1: Urban Driving (Single Floor, Moderate Speed)

Typical characteristics: Large-scale loops, moderate drift, flat terrain, some perceptual aliasing (similar-looking intersections).

| Parameter | Recommended | Rationale |
|---|---|---|
| `scan_context_distance_threshold` | **0.13** | Tighter to avoid aliasing between similar road segments |
| `loop_candidate_max_height_diff_m` | **1.0** | Flat terrain, 1m accounts for road grade and suspension |
| `icp_fitness_threshold` | **0.35** | Tighter to require high-quality alignment |
| `icp_max_correspondence_distance` | **5.0** | Slightly reduced; well-structured outdoor environment |
| `loop_candidate_max_yaw_diff_deg_from_odom` | **30.0** | SC yaw is quantized to 6 deg sectors; 30 deg gives ~5 sector tolerance |
| `loop_max_translation_delta_from_guess` | **2.5** | Tighter; odometry is usually reliable outdoors |
| `loop_max_rotation_delta_deg_from_guess` | **15.0** | Tighter for same reason |
| `keyframe_translation_thresh` | **1.5** | Wider spacing reduces computation, acceptable for driving speed |

#### Scenario 2: Indoor / Warehouse (Single Floor, Slow Speed)

Typical characteristics: Small loops, low drift, highly repetitive structures (shelving aisles), many potential false positives.

| Parameter | Recommended | Rationale |
|---|---|---|
| `scan_context_distance_threshold` | **0.10** | Very tight to combat perceptual aliasing in repetitive environments |
| `loop_candidate_max_height_diff_m` | **0.5** | Single floor, minimal elevation change |
| `icp_fitness_threshold` | **0.25** | Very tight; indoor scans have dense, clean points |
| `icp_max_correspondence_distance` | **3.0** | Reduced; small environment, dense points |
| `loop_candidate_max_yaw_diff_deg_from_odom` | **25.0** | Tighter; slow motion means more accurate odometry |
| `loop_max_translation_delta_from_guess` | **1.5** | Tight; small-scale environment with good odometry |
| `loop_max_rotation_delta_deg_from_guess` | **10.0** | Tight for same reason |
| `loop_submap_num_keyframes_each_side` | **15** | Larger submap helps distinguish repetitive structures |

#### Scenario 3: Multi-Floor / Hilly Terrain

Typical characteristics: Overlapping XY positions at different elevations, high false positive risk from vertical aliasing.

| Parameter | Recommended | Rationale |
|---|---|---|
| `scan_context_distance_threshold` | **0.12** | Very conservative to avoid cross-floor matches |
| `loop_candidate_max_height_diff_m` | **3.0** | Allow legitimate loops on slopes/ramps, but filter floors |
| `icp_fitness_threshold` | **0.30** | Tight; cross-floor matches will have poor fitness |
| `loop_max_translation_delta_from_guess` | **4.0** | Relaxed; elevation changes may cause larger delta |
| `loop_max_rotation_delta_deg_from_guess` | **25.0** | Relaxed for same reason |

#### Scenario 4: Fast-Moving Vehicle / Aggressive Motion

Typical characteristics: High odometry drift, motion blur, sparse keyframes.

| Parameter | Recommended | Rationale |
|---|---|---|
| `scan_context_distance_threshold` | **0.20** | Slightly relaxed to compensate for noisier descriptors |
| `loop_candidate_max_height_diff_m` | **2.0** | Account for vehicle suspension and road unevenness |
| `icp_fitness_threshold` | **0.50** | Relaxed; scans are noisier at high speed |
| `icp_max_correspondence_distance` | **8.0** | Larger to handle bigger initial alignment error |
| `loop_candidate_max_yaw_diff_deg_from_odom` | **45.0** | Relaxed; odometry less reliable at high speed |
| `loop_max_translation_delta_from_guess` | **5.0** | Relaxed; more drift means larger correction needed |
| `loop_max_rotation_delta_deg_from_guess` | **30.0** | Relaxed for same reason |
| `keyframe_translation_thresh` | **0.5** | Denser keyframes to maintain loop detection quality |
| `force_keyframe_every_n` | **15** | Ensure keyframes even during fast straight-line motion |

### Recommended Configurations

Below are complete recommended `back_end` configurations for three profiles.

#### Conservative (Minimize False Positives)

Best for: safety-critical applications, multi-floor environments, repetitive structures.

```yaml
back_end:
  scan_context_distance_threshold: 0.10
  loop_candidate_max_height_diff_m: 0.8
  loop_candidate_max_yaw_diff_deg_from_odom: 25.0
  icp_max_iterations: 50
  icp_max_correspondence_distance: 3.0
  icp_fitness_threshold: 0.25
  loop_max_translation_delta_from_guess: 1.5
  loop_max_rotation_delta_deg_from_guess: 10.0
  loop_translation_information: 20.0
  loop_rotation_information: 30.0
```

#### Balanced (Default Recommendation)

Best for: general outdoor mapping, moderate environments.

```yaml
back_end:
  scan_context_distance_threshold: 0.15
  loop_candidate_max_height_diff_m: 1.5
  loop_candidate_max_yaw_diff_deg_from_odom: 35.0
  icp_max_iterations: 30
  icp_max_correspondence_distance: 5.0
  icp_fitness_threshold: 0.40
  loop_max_translation_delta_from_guess: 3.0
  loop_max_rotation_delta_deg_from_guess: 20.0
  loop_translation_information: 30.0
  loop_rotation_information: 50.0
```

#### Aggressive (Maximize Loop Detection Recall)

Best for: large-scale outdoor mapping where drift correction is critical and the environment has distinctive structures.

```yaml
back_end:
  scan_context_distance_threshold: 0.22
  loop_candidate_max_height_diff_m: 3.0
  loop_candidate_max_yaw_diff_deg_from_odom: 45.0
  icp_max_iterations: 30
  icp_max_correspondence_distance: 8.0
  icp_fitness_threshold: 0.60
  loop_max_translation_delta_from_guess: 5.0
  loop_max_rotation_delta_deg_from_guess: 30.0
  loop_translation_information: 40.0
  loop_rotation_information: 60.0
```

### Diagnostic Workflow

When loop closures are not working as expected, follow this systematic diagnostic process:

#### Step 1: Enable logging

Set `enable_record_raw: true` and `enable_record_optimized: true` in the config. The backend writes anomaly logs to `backend_anomalies.txt` and the frontend writes to `frontend_publish_anomalies.txt`.

#### Step 2: Check keyframe generation

Monitor the `keyframe_id` in `BackendProcessResult`. If keyframes are too sparse, loops may be missed because `NUM_EXCLUDE_RECENT = 30` keyframes are always excluded.

**Symptom**: No loop candidates detected at all.
**Fix**: Lower `keyframe_translation_thresh` or `force_keyframe_every_n`.

#### Step 3: Check Scan Context candidates

Watch for `[Loop found]` vs `[Not loop]` messages in stdout (from `Scancontext.cpp`). The printed distance tells you how close candidates are to the threshold.

**Symptom**: Candidates found but distances are just above the threshold.
**Fix**: Slightly increase `scan_context_distance_threshold` (e.g., by 0.02-0.03).

**Symptom**: Many candidates found with very low distances (< 0.05) but in wrong locations.
**Fix**: This is perceptual aliasing. Decrease `scan_context_distance_threshold` and tighten ICP gates.

#### Step 4: Check rejection reasons

The backend logs specific rejection reasons:

| Log Message | Stage | Action |
|---|---|---|
| `"Reject loop ... due to height gap"` | Height filter | Adjust `loop_candidate_max_height_diff_m` |
| `"Reject loop ... due to scan-context/odom yaw mismatch"` | Yaw check | Increase `loop_candidate_max_yaw_diff_deg_from_odom` |
| `"Reject loop ... because ICP did not converge"` | ICP | Increase `icp_max_iterations` or `icp_max_correspondence_distance` |
| `"Reject loop ... due to ICP fitness"` | ICP fitness | Increase `icp_fitness_threshold` (but beware of false positives) |
| `"Reject loop ... due to odom consistency check"` | Consistency | Increase `loop_max_translation_delta_from_guess` or `loop_max_rotation_delta_deg_from_guess` |

#### Step 5: Check for map distortion after accepted loops

If the map deforms after a loop closure, the accepted loop was likely a false positive.

**Fix**: Tighten `icp_fitness_threshold` first (most impactful). Then tighten `scan_context_distance_threshold`. Lower `loop_translation_information` and `loop_rotation_information` to reduce the impact of any remaining false positives.

#### Step 6: Information matrix balancing

The ratio of odometry information to loop information controls how much the optimizer trusts each source:

```
Current ratio: odom_trans(100) / loop_trans(30) = 3.3x more trust in odometry
Current ratio: odom_rot(150)  / loop_rot(50)  = 3.0x more trust in odometry
```

- If loops are correct but barely affect the trajectory: **increase** `loop_translation_information` and `loop_rotation_information`.
- If loops cause excessive trajectory deformation: **decrease** loop information or **increase** odometry information.
- For high-quality IESKF odometry (typical for this system), the 3:1 ratio is reasonable. For noisier odometry, consider 1.5:1 to 2:1.

---

## Changelog

### commit 877fd168 - Enhanced cloud_with_pose monitoring and loop candidate filtering

- Added `loop_candidate_max_height_diff_m` parameter for height-based loop candidate rejection
- Backend wrapper switched to async consumption of `cloud_with_pose` messages
- Added keyframe time interval warnings and anomaly logging
- Frontend wrapper added `cloud_with_pose` publish gap monitoring

### commit 862a84e0 - Complete back-end loop closure optimization pipeline

- Introduced Ceres-based pose-graph optimizer replacing the simple optimizer
- Added Scan Context distance threshold as configurable parameter (`scan_context_distance_threshold`)
- Added ICP registration with configurable thresholds (`icp_fitness_threshold`, `icp_max_correspondence_distance`, `icp_max_iterations`)
- Added yaw consistency check (`loop_candidate_max_yaw_diff_deg_from_odom`)
- Added odometry consistency gates (`loop_max_translation_delta_from_guess`, `loop_max_rotation_delta_deg_from_guess`)
- Added submap construction for ICP targets (`loop_submap_num_keyframes_each_side`, `loop_submap_voxel_leaf_size`)
- Added configurable information matrices for odometry and loop edges
- Added trajectory recording and dense map export support
