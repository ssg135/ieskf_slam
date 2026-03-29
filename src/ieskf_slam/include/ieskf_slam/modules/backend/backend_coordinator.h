#pragma once

#include "ieskf_slam/modules/backend/backend_types.h"
#include "ieskf_slam/modules/backend/icp_loop_registrar.h"
#include "ieskf_slam/modules/backend/keyframe_store.h"
#include "ieskf_slam/modules/backend/scan_context_loop_detector.h"
#include "ieskf_slam/modules/backend/simple_pose_graph_optimizer.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/base_type.h"

namespace IESKFSLAM {

class BackendCoordinator : private ModuleBase {
public:
    BackendCoordinator(const std::string& config_path, const std::string& prefix);

    BackendProcessResult processFrame(const PCLPointCloud& cloud, const Pose& raw_pose, double stamp_sec);
    std::vector<Pose> readOptimizedPoses() const;
    PCLPointCloud buildOptimizedMap() const;

private:
    bool shouldCreateKeyframe(const Pose& raw_pose);
    Keyframe makeKeyframe(const PCLPointCloud& cloud, const Pose& raw_pose, double stamp_sec) const;
    GraphEdge makeOdometryEdge(const Keyframe& previous_keyframe, const Keyframe& current_keyframe) const;
    Pose initialGuessFromPrevious(const Pose& raw_pose) const;

    double keyframe_translation_thresh_;
    double keyframe_rotation_thresh_rad_;
    int force_keyframe_every_n_;
    double keyframe_voxel_leaf_size_;
    double map_voxel_leaf_size_;
    double odom_translation_information_;
    double odom_rotation_information_;

    KeyframeStore keyframe_store_;
    ScanContextLoopDetector loop_detector_;
    ICPLoopRegistrar loop_registrar_;
    SimplePoseGraphOptimizer pose_graph_optimizer_;
    mutable VoxelFilter map_voxel_filter_;
    VoxelFilter keyframe_voxel_filter_;
    int frames_since_last_keyframe_ = 0;
    int last_keyframe_id_ = -1;
};

}  // namespace IESKFSLAM
