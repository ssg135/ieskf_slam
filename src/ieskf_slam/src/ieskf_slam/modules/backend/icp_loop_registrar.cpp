#include "ieskf_slam/modules/backend/icp_loop_registrar.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <pcl/registration/icp.h>

namespace IESKFSLAM {

ICPLoopRegistrar::ICPLoopRegistrar(int max_iterations, double max_correspondence_distance,
                                   double fitness_threshold, double translation_information_scale,
                                   double rotation_information_scale)
    : max_iterations_(max_iterations),
      max_correspondence_distance_(max_correspondence_distance),
      fitness_threshold_(fitness_threshold),
      translation_information_scale_(translation_information_scale),
      rotation_information_scale_(rotation_information_scale) {}

LoopConstraint ICPLoopRegistrar::registerLoop(const Keyframe& query_keyframe,
                                              const Keyframe& target_keyframe,
                                              const LoopCandidate& candidate) {
    LoopConstraint constraint;
    if (!candidate.valid) {
        return constraint;
    }

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(query_keyframe.downsampled_cloud.makeShared());
    icp.setInputTarget(target_keyframe.downsampled_cloud.makeShared());

    Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    initial_transform.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(static_cast<float>(-candidate.yaw_init_rad), Eigen::Vector3f::UnitZ())
            .toRotationMatrix();

    PCLPointCloud aligned_cloud;
    icp.align(aligned_cloud, initial_transform);
    if (!icp.hasConverged()) {
        return constraint;
    }

    const double fitness = icp.getFitnessScore();
    if (fitness > fitness_threshold_) {
        return constraint;
    }

    const Eigen::Matrix4d transform = icp.getFinalTransformation().cast<double>();
    constraint.from_id = target_keyframe.id;
    constraint.to_id = query_keyframe.id;
    constraint.relative_pose = isometryToPose(Eigen::Isometry3d(transform));
    constraint.fitness = fitness;
    constraint.valid = true;

    const double information_scale = 1.0 / std::max(fitness, 1e-3);
    constraint.information.setZero();
    constraint.information.topLeftCorner<3, 3>() =
        Matrix6d::Identity().topLeftCorner<3, 3>() * translation_information_scale_ * information_scale;
    constraint.information.bottomRightCorner<3, 3>() =
        Matrix6d::Identity().bottomRightCorner<3, 3>() * rotation_information_scale_ * information_scale;
    return constraint;
}

}  // namespace IESKFSLAM
