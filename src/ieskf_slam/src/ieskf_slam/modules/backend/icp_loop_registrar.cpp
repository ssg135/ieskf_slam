#include "ieskf_slam/modules/backend/icp_loop_registrar.h"
#include "ieskf_slam/common/logging.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <pcl/registration/icp.h>

namespace IESKFSLAM {

ICPLoopRegistrar::ICPLoopRegistrar(int max_iterations, double max_correspondence_distance,
                                   double fitness_threshold, double translation_information_scale,
                                   double rotation_information_scale,
                                   double max_candidate_yaw_diff_rad,
                                   double max_translation_delta_from_guess,
                                   double max_rotation_delta_from_guess_rad)
    : max_iterations_(max_iterations),
      max_correspondence_distance_(max_correspondence_distance),
      fitness_threshold_(fitness_threshold),
      translation_information_scale_(translation_information_scale),
      rotation_information_scale_(rotation_information_scale),
      max_candidate_yaw_diff_rad_(max_candidate_yaw_diff_rad),
      max_translation_delta_from_guess_(max_translation_delta_from_guess),
      max_rotation_delta_from_guess_rad_(max_rotation_delta_from_guess_rad) {}

LoopConstraint ICPLoopRegistrar::registerLoop(const Keyframe& query_keyframe,
                                              const Keyframe& target_keyframe,
                                              const LoopCandidate& candidate) {
    LoopConstraint constraint;
    if (!candidate.valid) {
        return constraint;
    }

    const Pose initial_guess_pose =
        relativePose(target_keyframe.optimized_pose, query_keyframe.optimized_pose);
    const double odom_guess_yaw = yawFromQuaternion(initial_guess_pose.rotation);
    const double scan_context_yaw = -candidate.yaw_init_rad;
    const double candidate_yaw_delta = angleDistanceRad(odom_guess_yaw, scan_context_yaw);
    if (candidate_yaw_delta > max_candidate_yaw_diff_rad_) {
        SLAM_LOG_WARN << "Reject loop " << target_keyframe.id << " -> " << query_keyframe.id
                      << " due to scan-context/odom yaw mismatch: "
                      << candidate_yaw_delta * 180.0 / M_PI << " deg";
        return constraint;
    }

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputSource(query_keyframe.downsampled_cloud.makeShared());
    icp.setInputTarget(target_keyframe.downsampled_cloud.makeShared());

    const Eigen::Matrix4f initial_transform =
        poseToIsometry(initial_guess_pose).matrix().cast<float>();

    PCLPointCloud aligned_cloud;
    icp.align(aligned_cloud, initial_transform);
    if (!icp.hasConverged()) {
        SLAM_LOG_WARN << "Reject loop " << target_keyframe.id << " -> " << query_keyframe.id
                      << " because ICP did not converge";
        return constraint;
    }

    const double fitness = icp.getFitnessScore();
    if (fitness > fitness_threshold_) {
        SLAM_LOG_WARN << "Reject loop " << target_keyframe.id << " -> " << query_keyframe.id
                      << " due to ICP fitness " << fitness;
        return constraint;
    }

    const Eigen::Matrix4d transform = icp.getFinalTransformation().cast<double>();
    const Pose refined_loop_pose = isometryToPose(Eigen::Isometry3d(transform), query_keyframe.stamp_sec);
    const double translation_delta =
        (refined_loop_pose.position - initial_guess_pose.position).norm();
    const double rotation_delta = angleDistanceRad(
        yawFromQuaternion(refined_loop_pose.rotation), odom_guess_yaw);
    if (translation_delta > max_translation_delta_from_guess_ ||
        rotation_delta > max_rotation_delta_from_guess_rad_) {
        SLAM_LOG_WARN << "Reject loop " << target_keyframe.id << " -> " << query_keyframe.id
                      << " due to odom consistency check. translation_delta=" << translation_delta
                      << " m, rotation_delta=" << rotation_delta * 180.0 / M_PI << " deg";
        return constraint;
    }

    constraint.from_id = target_keyframe.id;
    constraint.to_id = query_keyframe.id;
    constraint.relative_pose = refined_loop_pose;
    constraint.fitness = fitness;
    constraint.valid = true;

    const double information_scale = 1.0 / std::max(fitness, 1e-3);
    constraint.information.setZero();
    constraint.information.topLeftCorner<3, 3>() =
        Matrix6d::Identity().topLeftCorner<3, 3>() * translation_information_scale_ * information_scale;
    constraint.information.bottomRightCorner<3, 3>() =
        Matrix6d::Identity().bottomRightCorner<3, 3>() * rotation_information_scale_ * information_scale;
    SLAM_LOG_INFO << "Accept loop " << target_keyframe.id << " -> " << query_keyframe.id
                  << ", fitness=" << fitness
                  << ", translation_delta=" << translation_delta
                  << ", rotation_delta_deg=" << rotation_delta * 180.0 / M_PI;
    return constraint;
}

}  // namespace IESKFSLAM
