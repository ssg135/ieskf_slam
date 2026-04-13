#pragma once

#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include <Eigen/Dense>

namespace IESKFSLAM {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Keyframe {
    int id = -1;
    double stamp_sec = 0.0;
    Pose raw_pose;
    Pose optimized_pose;
    PCLPointCloud cloud;
    PCLPointCloud downsampled_cloud;
};

struct LoopCandidate {
    int query_id = -1;
    int target_id = -1;
    double score = 0.0;
    double yaw_init_rad = 0.0;
    bool valid = false;
};

struct LoopConstraint {
    int from_id = -1;
    int to_id = -1;
    Pose relative_pose;
    Matrix6d information = Matrix6d::Identity();
    double fitness = 0.0;
    bool valid = false;
};

struct GraphEdge {
    enum class Type {
        ODOM,
        LOOP,
    };

    int from_id = -1;
    int to_id = -1;
    Pose relative_pose;
    Matrix6d information = Matrix6d::Identity();
    Type type = Type::ODOM;
};

struct BackendProcessResult {
    bool inserted_keyframe = false;
    bool found_loop_candidate = false;
    bool accepted_loop = false;
    bool optimized = false;
    bool detected_large_keyframe_gap = false;
    int keyframe_id = -1;
    int loop_target_id = -1;
    int previous_keyframe_id = -1;
    int frames_since_last_keyframe = 0;
    double keyframe_gap_sec = 0.0;
    double keyframe_raw_translation = 0.0;
    double keyframe_optimized_translation = 0.0;
};

}  // namespace IESKFSLAM
