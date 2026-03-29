#pragma once

#include "ieskf_slam/math/SO3.h"
#include "ieskf_slam/modules/backend/backend_types.h"
#include "ieskf_slam/type/pose.h"
#include <Eigen/Geometry>
#include <cmath>

namespace IESKFSLAM {

inline Pose makePose(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& position,
                     double stamp_sec = 0.0) {
    Pose pose;
    pose.rotation = rotation.normalized();
    pose.position = position;
    pose.time_stamp.fromSec(stamp_sec);
    return pose;
}

inline Eigen::Isometry3d poseToIsometry(const Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = pose.rotation.normalized().toRotationMatrix();
    transform.translation() = pose.position;
    return transform;
}

inline Pose isometryToPose(const Eigen::Isometry3d& transform, double stamp_sec = 0.0) {
    return makePose(Eigen::Quaterniond(transform.linear()), transform.translation(), stamp_sec);
}

inline Pose inversePose(const Pose& pose) {
    const Eigen::Quaterniond inverse_rotation = pose.rotation.conjugate().normalized();
    return makePose(inverse_rotation, inverse_rotation * (-pose.position), pose.time_stamp.sec());
}

inline Pose composePose(const Pose& lhs, const Pose& rhs) {
    return makePose(lhs.rotation * rhs.rotation, lhs.position + lhs.rotation * rhs.position,
                    rhs.time_stamp.sec());
}

inline Pose relativePose(const Pose& from, const Pose& to) {
    return composePose(inversePose(from), to);
}

inline Vector6d poseResidual(const Pose& measurement, const Pose& prediction) {
    const Pose error_pose = composePose(inversePose(measurement), prediction);
    Vector6d residual = Vector6d::Zero();
    residual.head<3>() = error_pose.position;
    residual.tail<3>() = SO3Log(error_pose.rotation.toRotationMatrix());
    return residual;
}

inline Pose applyPoseIncrement(const Pose& pose, const Vector6d& increment) {
    const Eigen::Quaterniond delta_rotation(so3Exp(increment.tail<3>()));
    return makePose((pose.rotation * delta_rotation).normalized(),
                    pose.position + increment.head<3>(), pose.time_stamp.sec());
}

inline double yawFromQuaternion(const Eigen::Quaterniond& rotation) {
    const Eigen::Matrix3d matrix = rotation.toRotationMatrix();
    return std::atan2(matrix(1, 0), matrix(0, 0));
}

inline Pose yawOnlyPose(double yaw_rad) {
    return makePose(Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())),
                    Eigen::Vector3d::Zero());
}

}  // namespace IESKFSLAM
