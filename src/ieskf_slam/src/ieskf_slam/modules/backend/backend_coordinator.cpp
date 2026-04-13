#include "ieskf_slam/modules/backend/backend_coordinator.h"
#include "ieskf_slam/common/logging.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <algorithm>
#include <cmath>
#include <pcl/common/transforms.h>

namespace IESKFSLAM {

namespace {
constexpr double kWarnKeyframeGapSec = 5.0;
}

BackendCoordinator::BackendCoordinator(const std::string& config_path, const std::string& prefix)
    : ModuleBase(config_path, prefix, "BackEnd"),
      loop_registrar_(20, 6.0, 0.8, 30.0, 50.0, 35.0 * M_PI / 180.0, 3.0,
                      20.0 * M_PI / 180.0),
      pose_graph_optimizer_(15, 1e-4, 1e-6) {
    double keyframe_rotation_thresh_deg = 10.0;
    readParam("keyframe_translation_thresh", keyframe_translation_thresh_, 1.0);
    readParam("keyframe_rotation_thresh_deg", keyframe_rotation_thresh_deg, 10.0);
    readParam("force_keyframe_every_n", force_keyframe_every_n_, 30);
    readParam("keyframe_voxel_leaf_size", keyframe_voxel_leaf_size_, 0.5);
    readParam("map_voxel_leaf_size", map_voxel_leaf_size_, 0.5);
    readParam("dense_map_keyframe_voxel_leaf_size", dense_map_keyframe_voxel_leaf_size_, 0.3);
    readParam("dense_map_global_voxel_leaf_size", dense_map_global_voxel_leaf_size_, 0.3);
    readParam("loop_submap_voxel_leaf_size", loop_submap_voxel_leaf_size_, 0.5);
    readParam("map_visualization_radius", map_visualization_radius_, 30.0);
    readParam("map_visualization_min_recent_keyframes", map_visualization_min_recent_keyframes_, 20);
    readParam("map_visualization_max_keyframes", map_visualization_max_keyframes_, 80);
    readParam("loop_submap_num_keyframes_each_side", loop_submap_num_keyframes_each_side_, 10);
    readParam("enable_loop_closure", enable_loop_closure_, false);
    readParam("odom_translation_information", odom_translation_information_, 100.0);
    readParam("odom_rotation_information", odom_rotation_information_, 150.0);
    double scan_context_distance_threshold = 0.18;
    double loop_candidate_max_height_diff_m = 1.5;
    double loop_candidate_max_yaw_diff_deg_from_odom = 35.0;
    double loop_max_translation_delta_from_guess = 3.0;
    double loop_max_rotation_delta_deg_from_guess = 20.0;
    readParam("scan_context_distance_threshold", scan_context_distance_threshold, 0.18);
    readParam("loop_candidate_max_height_diff_m", loop_candidate_max_height_diff_m, 1.5);
    readParam("loop_candidate_max_yaw_diff_deg_from_odom",
              loop_candidate_max_yaw_diff_deg_from_odom, 35.0);
    readParam("loop_max_translation_delta_from_guess",
              loop_max_translation_delta_from_guess, 3.0);
    readParam("loop_max_rotation_delta_deg_from_guess",
              loop_max_rotation_delta_deg_from_guess, 20.0);

    int icp_max_iterations = 30;
    double icp_max_correspondence_distance = 6.0;
    double icp_fitness_threshold = 0.8;
    double loop_translation_information = 30.0;
    double loop_rotation_information = 50.0;
    readParam("icp_max_iterations", icp_max_iterations, 30);
    readParam("icp_max_correspondence_distance", icp_max_correspondence_distance, 6.0);
    readParam("icp_fitness_threshold", icp_fitness_threshold, 0.8);
    readParam("loop_translation_information", loop_translation_information, 30.0);
    readParam("loop_rotation_information", loop_rotation_information, 50.0);
    loop_registrar_ = ICPLoopRegistrar(icp_max_iterations, icp_max_correspondence_distance,
                                       icp_fitness_threshold, loop_translation_information,
                                       loop_rotation_information,
                                       loop_candidate_max_yaw_diff_deg_from_odom * M_PI / 180.0,
                                       loop_max_translation_delta_from_guess,
                                       loop_max_rotation_delta_deg_from_guess * M_PI / 180.0);
    loop_detector_.setDistanceThreshold(scan_context_distance_threshold);

    int optimizer_max_iterations = 15;
    double optimizer_stop_delta_norm = 1e-4;
    double optimizer_damping_lambda = 1e-6;
    readParam("optimizer_max_iterations", optimizer_max_iterations, 15);
    readParam("optimizer_stop_delta_norm", optimizer_stop_delta_norm, 1e-4);
    readParam("optimizer_damping_lambda", optimizer_damping_lambda, 1e-6);
    pose_graph_optimizer_ = CeresPoseGraphOptimizer(optimizer_max_iterations,
                                                    optimizer_stop_delta_norm,
                                                    optimizer_damping_lambda);

    keyframe_rotation_thresh_rad_ = keyframe_rotation_thresh_deg * M_PI / 180.0;
    loop_candidate_max_height_diff_m_ = std::max(loop_candidate_max_height_diff_m, 0.0);
    loop_submap_num_keyframes_each_side_ = std::max(loop_submap_num_keyframes_each_side_, 0);
    map_visualization_min_recent_keyframes_ = std::max(map_visualization_min_recent_keyframes_, 1);
    if (map_visualization_max_keyframes_ > 0) {
        map_visualization_max_keyframes_ =
            std::max(map_visualization_max_keyframes_, map_visualization_min_recent_keyframes_);
    }
    keyframe_voxel_filter_.setLeafSize(keyframe_voxel_leaf_size_, keyframe_voxel_leaf_size_,
                                       keyframe_voxel_leaf_size_);
    map_voxel_filter_.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
    dense_map_keyframe_voxel_filter_.setLeafSize(dense_map_keyframe_voxel_leaf_size_,
                                                 dense_map_keyframe_voxel_leaf_size_,
                                                 dense_map_keyframe_voxel_leaf_size_);
    dense_map_global_voxel_filter_.setLeafSize(dense_map_global_voxel_leaf_size_,
                                               dense_map_global_voxel_leaf_size_,
                                               dense_map_global_voxel_leaf_size_);
    print_table();
}

bool BackendCoordinator::isLoopClosureEnabled() const {
    return enable_loop_closure_;
}

bool BackendCoordinator::shouldCreateKeyframe(const Pose& raw_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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

Keyframe BackendCoordinator::buildLoopSubmapKeyframe(int target_keyframe_id) const {
    const Keyframe& target_keyframe = keyframe_store_.get(target_keyframe_id);

    Keyframe submap_keyframe;
    submap_keyframe.id = target_keyframe.id;
    submap_keyframe.stamp_sec = target_keyframe.stamp_sec;
    submap_keyframe.raw_pose = target_keyframe.raw_pose;
    submap_keyframe.optimized_pose = target_keyframe.optimized_pose;

    const auto& keyframes = keyframe_store_.all();
    const int first_index = std::max(0, target_keyframe_id - loop_submap_num_keyframes_each_side_);
    const int last_index = std::min(static_cast<int>(keyframes.size()) - 1,
                                    target_keyframe_id + loop_submap_num_keyframes_each_side_);
    for (int index = first_index; index <= last_index; ++index) {
        PCLPointCloud transformed_cloud = keyframes[index].downsampled_cloud;
        if (transformed_cloud.empty()) {
            continue;
        }

        pcl::transformPointCloud(
            transformed_cloud, transformed_cloud,
            poseToIsometry(relativePose(target_keyframe.optimized_pose, keyframes[index].optimized_pose))
                .matrix()
                .cast<float>());
        submap_keyframe.downsampled_cloud += transformed_cloud;
    }

    if (submap_keyframe.downsampled_cloud.empty()) {
        submap_keyframe.downsampled_cloud = target_keyframe.downsampled_cloud;
        return submap_keyframe;
    }

    if (loop_submap_voxel_leaf_size_ > 0.0) {
        VoxelFilter voxel_filter = map_voxel_filter_;
        voxel_filter.setLeafSize(loop_submap_voxel_leaf_size_, loop_submap_voxel_leaf_size_,
                                 loop_submap_voxel_leaf_size_);
        voxel_filter.setInputCloud(submap_keyframe.downsampled_cloud.makeShared());

        PCLPointCloud filtered_submap;
        voxel_filter.filter(filtered_submap);
        submap_keyframe.downsampled_cloud = std::move(filtered_submap);
    }

    return submap_keyframe;
}

BackendProcessResult BackendCoordinator::processKeyframe(const PCLPointCloud& cloud, const Pose& raw_pose,
                                                         double stamp_sec) {
    BackendProcessResult result;
    Keyframe keyframe = makeKeyframe(cloud, raw_pose, stamp_sec);
    std::lock_guard<std::mutex> lock(mutex_);
    const int previous_keyframe_id = last_keyframe_id_;
    const int keyframe_id = keyframe_store_.addKeyframe(std::move(keyframe));
    Keyframe& current_keyframe = keyframe_store_.mutableGet(keyframe_id);
    current_keyframe.id = keyframe_id;
    result.inserted_keyframe = true;
    result.keyframe_id = keyframe_id;
    const int frames_since_last_keyframe = frames_since_last_keyframe_;
    frames_since_last_keyframe_ = 0;

    pose_graph_optimizer_.addNode(keyframe_id, current_keyframe.optimized_pose);
    if (previous_keyframe_id >= 0) {
        const Keyframe& previous_keyframe = keyframe_store_.get(previous_keyframe_id);
        const double keyframe_gap_sec = current_keyframe.stamp_sec - previous_keyframe.stamp_sec;
        if (keyframe_gap_sec > kWarnKeyframeGapSec) {
            const double raw_translation =
                (current_keyframe.raw_pose.position - previous_keyframe.raw_pose.position).norm();
            const double optimized_translation =
                (current_keyframe.optimized_pose.position - previous_keyframe.optimized_pose.position).norm();
            result.detected_large_keyframe_gap = true;
            result.previous_keyframe_id = previous_keyframe.id;
            result.frames_since_last_keyframe = frames_since_last_keyframe;
            result.keyframe_gap_sec = keyframe_gap_sec;
            result.keyframe_raw_translation = raw_translation;
            result.keyframe_optimized_translation = optimized_translation;
            SLAM_LOG_WARN << "Large keyframe time gap before insert. prev_id=" << previous_keyframe.id
                          << ", curr_id=" << current_keyframe.id
                          << ", dt=" << keyframe_gap_sec
                          << " s, frames_since_last_keyframe=" << frames_since_last_keyframe
                          << ", raw_translation=" << raw_translation
                          << " m, optimized_translation=" << optimized_translation << " m";
        }
        pose_graph_optimizer_.addEdge(makeOdometryEdge(previous_keyframe, current_keyframe));
    }

    last_keyframe_id_ = keyframe_id;
    return result;
}

BackendProcessResult BackendCoordinator::processLoopClosure(int keyframe_id) {
    BackendProcessResult result;
    result.keyframe_id = keyframe_id;
    if (!enable_loop_closure_) {
        return result;
    }

    Keyframe current_keyframe;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (keyframe_id < 0 || keyframe_id >= static_cast<int>(keyframe_store_.size())) {
            return result;
        }
        current_keyframe = keyframe_store_.get(keyframe_id);
    }

    loop_detector_.addKeyframe(current_keyframe);
    const LoopCandidate candidate = loop_detector_.detect(current_keyframe);
    if (!candidate.valid) {
        return result;
    }

    result.found_loop_candidate = true;
    result.loop_target_id = candidate.target_id;

    Keyframe target_keyframe;
    Keyframe target_submap_keyframe;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (candidate.target_id < 0 || candidate.target_id >= static_cast<int>(keyframe_store_.size())) {
            return result;
        }

        target_keyframe = keyframe_store_.get(candidate.target_id);
        const double loop_height_delta =
            std::abs(current_keyframe.optimized_pose.position.z() - target_keyframe.optimized_pose.position.z());
        if (loop_candidate_max_height_diff_m_ > 0.0 &&
            loop_height_delta > loop_candidate_max_height_diff_m_) {
            SLAM_LOG_WARN << "Reject loop candidate " << candidate.target_id << " -> "
                          << current_keyframe.id << " due to height gap: "
                          << loop_height_delta << " m";
            return result;
        }

        target_submap_keyframe = buildLoopSubmapKeyframe(candidate.target_id);
    }

    const LoopConstraint loop_constraint =
        loop_registrar_.registerLoop(current_keyframe, target_submap_keyframe, candidate);
    if (!loop_constraint.valid) {
        return result;
    }

    GraphEdge loop_edge;
    loop_edge.from_id = loop_constraint.from_id;
    loop_edge.to_id = loop_constraint.to_id;
    loop_edge.relative_pose = loop_constraint.relative_pose;
    loop_edge.information = loop_constraint.information;
    loop_edge.type = GraphEdge::Type::LOOP;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_graph_optimizer_.addEdge(loop_edge);
        result.accepted_loop = true;

        if (pose_graph_optimizer_.optimize()) {
            keyframe_store_.updateOptimizedPoses(pose_graph_optimizer_.nodePoses());
            result.optimized = true;
        } else {
            pose_graph_optimizer_.removeLastEdge();
        }
    }

    return result;
}

std::vector<Pose> BackendCoordinator::readOptimizedPoses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Pose> poses;
    poses.reserve(keyframe_store_.size());
    for (const auto& keyframe : keyframe_store_.all()) {
        poses.push_back(keyframe.optimized_pose);
    }
    return poses;
}

PCLPointCloud BackendCoordinator::buildOptimizedMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    PCLPointCloud aggregated_map;
    const auto& keyframes = keyframe_store_.all();
    if (keyframes.empty()) {
        return aggregated_map;
    }

    const Pose& latest_pose = keyframes.back().optimized_pose;
    const std::size_t keyframe_count = keyframes.size();
    const std::size_t recent_window =
        static_cast<std::size_t>(std::max(map_visualization_min_recent_keyframes_, 1));
    const std::size_t first_recent_index =
        keyframe_count > recent_window ? keyframe_count - recent_window : 0;

    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(keyframe_count);
    for (std::size_t i = 0; i < keyframe_count; ++i) {
        const auto& keyframe = keyframes[i];
        const bool is_recent = i >= first_recent_index;
        const bool is_nearby =
            map_visualization_radius_ <= 0.0 ||
            (keyframe.optimized_pose.position - latest_pose.position).norm() <= map_visualization_radius_;
        if (is_recent || is_nearby) {
            selected_indices.push_back(i);
        }
    }

    if (selected_indices.empty()) {
        selected_indices.push_back(keyframe_count - 1);
    }

    if (map_visualization_max_keyframes_ > 0 &&
        selected_indices.size() > static_cast<std::size_t>(map_visualization_max_keyframes_)) {
        const std::size_t erase_count =
            selected_indices.size() - static_cast<std::size_t>(map_visualization_max_keyframes_);
        selected_indices.erase(selected_indices.begin(), selected_indices.begin() + erase_count);
    }

    for (const std::size_t index : selected_indices) {
        const auto& keyframe = keyframes[index];
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

PCLPointCloud BackendCoordinator::buildDenseOptimizedMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    PCLPointCloud dense_map;
    const auto& keyframes = keyframe_store_.all();
    if (keyframes.empty()) {
        return dense_map;
    }

    for (const auto& keyframe : keyframes) {
        PCLPointCloud filtered_keyframe_cloud = keyframe.cloud;
        if (!filtered_keyframe_cloud.empty() && dense_map_keyframe_voxel_leaf_size_ > 0.0) {
            VoxelFilter voxel_filter = dense_map_keyframe_voxel_filter_;
            voxel_filter.setInputCloud(filtered_keyframe_cloud.makeShared());
            PCLPointCloud downsampled_keyframe_cloud;
            voxel_filter.filter(downsampled_keyframe_cloud);
            filtered_keyframe_cloud = std::move(downsampled_keyframe_cloud);
        }

        PCLPointCloud transformed_cloud = std::move(filtered_keyframe_cloud);
        pcl::transformPointCloud(transformed_cloud, transformed_cloud,
                                 poseToIsometry(keyframe.optimized_pose).matrix().cast<float>());
        dense_map += transformed_cloud;
    }

    if (dense_map.empty() || dense_map_global_voxel_leaf_size_ <= 0.0) {
        return dense_map;
    }

    PCLPointCloud filtered_dense_map;
    VoxelFilter voxel_filter = dense_map_global_voxel_filter_;
    voxel_filter.setInputCloud(dense_map.makeShared());
    voxel_filter.filter(filtered_dense_map);
    return filtered_dense_map;
}

}  // namespace IESKFSLAM
