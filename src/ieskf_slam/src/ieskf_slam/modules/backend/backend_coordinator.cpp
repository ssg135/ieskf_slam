#include "ieskf_slam/modules/backend/backend_coordinator.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <cmath>
#include <pcl/common/transforms.h>

namespace IESKFSLAM {

BackendCoordinator::BackendCoordinator(const std::string& config_path, const std::string& prefix)
    : ModuleBase(config_path, prefix, "BackEnd"),
      loop_registrar_(20, 15.0, 1.5, 30.0, 50.0),
      pose_graph_optimizer_(15, 1e-4, 1e-6) {
    double keyframe_rotation_thresh_deg = 10.0;
    readParam("keyframe_translation_thresh", keyframe_translation_thresh_, 1.0);
    readParam("keyframe_rotation_thresh_deg", keyframe_rotation_thresh_deg, 10.0);
    readParam("force_keyframe_every_n", force_keyframe_every_n_, 30);
    readParam("keyframe_voxel_leaf_size", keyframe_voxel_leaf_size_, 0.5);
    readParam("map_voxel_leaf_size", map_voxel_leaf_size_, 0.5);
    readParam("odom_translation_information", odom_translation_information_, 100.0);
    readParam("odom_rotation_information", odom_rotation_information_, 150.0);

    int icp_max_iterations = 30;
    double icp_max_correspondence_distance = 15.0;
    double icp_fitness_threshold = 1.5;
    double loop_translation_information = 30.0;
    double loop_rotation_information = 50.0;
    readParam("icp_max_iterations", icp_max_iterations, 30);
    readParam("icp_max_correspondence_distance", icp_max_correspondence_distance, 15.0);
    readParam("icp_fitness_threshold", icp_fitness_threshold, 1.5);
    readParam("loop_translation_information", loop_translation_information, 30.0);
    readParam("loop_rotation_information", loop_rotation_information, 50.0);
    loop_registrar_ = ICPLoopRegistrar(icp_max_iterations, icp_max_correspondence_distance,
                                       icp_fitness_threshold, loop_translation_information,
                                       loop_rotation_information);

    int optimizer_max_iterations = 15;
    double optimizer_stop_delta_norm = 1e-4;
    double optimizer_damping_lambda = 1e-6;
    readParam("optimizer_max_iterations", optimizer_max_iterations, 15);
    readParam("optimizer_stop_delta_norm", optimizer_stop_delta_norm, 1e-4);
    readParam("optimizer_damping_lambda", optimizer_damping_lambda, 1e-6);
    pose_graph_optimizer_ = SimplePoseGraphOptimizer(optimizer_max_iterations,
                                                     optimizer_stop_delta_norm,
                                                     optimizer_damping_lambda);

    keyframe_rotation_thresh_rad_ = keyframe_rotation_thresh_deg * M_PI / 180.0;
    keyframe_voxel_filter_.setLeafSize(keyframe_voxel_leaf_size_, keyframe_voxel_leaf_size_,
                                       keyframe_voxel_leaf_size_);
    map_voxel_filter_.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
    print_table();
}

bool BackendCoordinator::shouldCreateKeyframe(const Pose& raw_pose) {
    if (keyframe_store_.empty()) {
        return true;
    }

    ++frames_since_last_keyframe_;
    const Keyframe& last_keyframe = keyframe_store_.get(last_keyframe_id_);
    const double translation = (raw_pose.position - last_keyframe.raw_pose.position).norm();
    const Pose delta_pose = relativePose(last_keyframe.raw_pose, raw_pose);
    const double rotation = SO3Log(delta_pose.rotation.toRotationMatrix()).norm();
    return translation >= keyframe_translation_thresh_ ||
           rotation >= keyframe_rotation_thresh_rad_ ||
           frames_since_last_keyframe_ >= force_keyframe_every_n_;
}

Pose BackendCoordinator::initialGuessFromPrevious(const Pose& raw_pose) const {
    if (keyframe_store_.empty()) {
        return raw_pose;
    }
    const Keyframe& last_keyframe = keyframe_store_.get(last_keyframe_id_);
    return composePose(last_keyframe.optimized_pose, relativePose(last_keyframe.raw_pose, raw_pose));
}

Keyframe BackendCoordinator::makeKeyframe(const PCLPointCloud& cloud, const Pose& raw_pose,
                                          double stamp_sec) const {
    Keyframe keyframe;
    keyframe.stamp_sec = stamp_sec;
    keyframe.raw_pose = raw_pose;
    keyframe.optimized_pose = initialGuessFromPrevious(raw_pose);
    keyframe.cloud = cloud;
    keyframe.downsampled_cloud = cloud;

    VoxelFilter voxel_filter = keyframe_voxel_filter_;
    voxel_filter.setInputCloud(keyframe.downsampled_cloud.makeShared());
    voxel_filter.filter(keyframe.downsampled_cloud);
    return keyframe;
}

GraphEdge BackendCoordinator::makeOdometryEdge(const Keyframe& previous_keyframe,
                                               const Keyframe& current_keyframe) const {
    GraphEdge edge;
    edge.from_id = previous_keyframe.id;
    edge.to_id = current_keyframe.id;
    edge.relative_pose = relativePose(previous_keyframe.raw_pose, current_keyframe.raw_pose);
    edge.information.setZero();
    edge.information.topLeftCorner<3, 3>() =
        Eigen::Matrix3d::Identity() * odom_translation_information_;
    edge.information.bottomRightCorner<3, 3>() =
        Eigen::Matrix3d::Identity() * odom_rotation_information_;
    edge.type = GraphEdge::Type::ODOM;
    return edge;
}

BackendProcessResult BackendCoordinator::processFrame(const PCLPointCloud& cloud, const Pose& raw_pose,
                                                      double stamp_sec) {
    BackendProcessResult result;
    if (!shouldCreateKeyframe(raw_pose)) {
        return result;
    }

    Keyframe keyframe = makeKeyframe(cloud, raw_pose, stamp_sec);
    const int previous_keyframe_id = last_keyframe_id_;
    const int keyframe_id = keyframe_store_.addKeyframe(std::move(keyframe));
    Keyframe& current_keyframe = keyframe_store_.mutableGet(keyframe_id);
    current_keyframe.id = keyframe_id;
    result.inserted_keyframe = true;
    result.keyframe_id = keyframe_id;
    frames_since_last_keyframe_ = 0;

    pose_graph_optimizer_.addNode(keyframe_id, current_keyframe.optimized_pose);
    if (previous_keyframe_id >= 0) {
        pose_graph_optimizer_.addEdge(
            makeOdometryEdge(keyframe_store_.get(previous_keyframe_id), current_keyframe));
    }

    loop_detector_.addKeyframe(current_keyframe);
    const LoopCandidate candidate = loop_detector_.detect(current_keyframe);
    if (candidate.valid) {
        result.found_loop_candidate = true;
        result.loop_target_id = candidate.target_id;
        const LoopConstraint loop_constraint = loop_registrar_.registerLoop(
            current_keyframe, keyframe_store_.get(candidate.target_id), candidate);
        if (loop_constraint.valid) {
            GraphEdge loop_edge;
            loop_edge.from_id = loop_constraint.from_id;
            loop_edge.to_id = loop_constraint.to_id;
            loop_edge.relative_pose = loop_constraint.relative_pose;
            loop_edge.information = loop_constraint.information;
            loop_edge.type = GraphEdge::Type::LOOP;
            pose_graph_optimizer_.addEdge(loop_edge);
            result.accepted_loop = true;

            if (pose_graph_optimizer_.optimize()) {
                keyframe_store_.updateOptimizedPoses(pose_graph_optimizer_.nodePoses());
                result.optimized = true;
            } else {
                pose_graph_optimizer_.removeLastEdge();
            }
        }
    }

    last_keyframe_id_ = keyframe_id;
    return result;
}

std::vector<Pose> BackendCoordinator::readOptimizedPoses() const {
    std::vector<Pose> poses;
    poses.reserve(keyframe_store_.size());
    for (const auto& keyframe : keyframe_store_.all()) {
        poses.push_back(keyframe.optimized_pose);
    }
    return poses;
}

PCLPointCloud BackendCoordinator::buildOptimizedMap() const {
    PCLPointCloud aggregated_map;
    for (const auto& keyframe : keyframe_store_.all()) {
        PCLPointCloud transformed_cloud = keyframe.downsampled_cloud;
        pcl::transformPointCloud(transformed_cloud, transformed_cloud,
                                 poseToIsometry(keyframe.optimized_pose).matrix().cast<float>());
        aggregated_map += transformed_cloud;
    }

    if (aggregated_map.empty()) {
        return aggregated_map;
    }

    PCLPointCloud filtered_map;
    VoxelFilter voxel_filter = map_voxel_filter_;
    voxel_filter.setInputCloud(aggregated_map.makeShared());
    voxel_filter.filter(filtered_map);
    return filtered_map;
}

}  // namespace IESKFSLAM
