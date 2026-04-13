#include "ieskf_slam/modules/backend/ceres_pose_graph_optimizer.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <stdexcept>

namespace IESKFSLAM {

namespace {

Matrix6d computeSqrtInformation(const Matrix6d& information) {
    const Eigen::LLT<Matrix6d> llt(information);
    if (llt.info() == Eigen::Success) {
        return llt.matrixU();
    }

    const Eigen::SelfAdjointEigenSolver<Matrix6d> eigen_solver(information);
    if (eigen_solver.info() != Eigen::Success) {
        return Matrix6d::Identity();
    }

    const Eigen::Matrix<double, 6, 1> clamped_eigenvalues =
        eigen_solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
    return eigen_solver.eigenvectors() * clamped_eigenvalues.asDiagonal() *
           eigen_solver.eigenvectors().transpose();
}

struct PoseGraphEdgeCostFunctor {
    PoseGraphEdgeCostFunctor(const Pose& relative_pose, const Matrix6d& information)
        : measurement_rotation(relative_pose.rotation.normalized()),
          measurement_translation(relative_pose.position),
          sqrt_information(computeSqrtInformation(information)) {}

    template <typename T>
    bool operator()(const T* const from_quaternion_xyzw, const T* const from_translation_xyz,
                    const T* const to_quaternion_xyzw, const T* const to_translation_xyz,
                    T* residuals) const {
        const Eigen::Quaternion<T> from_rotation(from_quaternion_xyzw[3], from_quaternion_xyzw[0],
                                                 from_quaternion_xyzw[1], from_quaternion_xyzw[2]);
        const Eigen::Quaternion<T> to_rotation(to_quaternion_xyzw[3], to_quaternion_xyzw[0],
                                               to_quaternion_xyzw[1], to_quaternion_xyzw[2]);

        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> from_translation(from_translation_xyz);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> to_translation(to_translation_xyz);

        const Eigen::Quaternion<T> predicted_rotation =
            (from_rotation.conjugate() * to_rotation).normalized();
        const Eigen::Matrix<T, 3, 1> predicted_translation =
            from_rotation.conjugate() * (to_translation - from_translation);

        const Eigen::Quaternion<T> measured_rotation(
            T(measurement_rotation.w()), T(measurement_rotation.x()), T(measurement_rotation.y()),
            T(measurement_rotation.z()));
        const Eigen::Matrix<T, 3, 1> measured_translation =
            measurement_translation.template cast<T>();

        const Eigen::Quaternion<T> rotation_error =
            (measured_rotation.conjugate() * predicted_rotation).normalized();

        T rotation_error_wxyz[4] = {rotation_error.w(), rotation_error.x(), rotation_error.y(),
                                    rotation_error.z()};
        T rotation_error_angle_axis[3];
        ceres::QuaternionToAngleAxis(rotation_error_wxyz, rotation_error_angle_axis);

        Eigen::Matrix<T, 6, 1> residual = Eigen::Matrix<T, 6, 1>::Zero();
        residual.template head<3>() = predicted_translation - measured_translation;
        residual.template tail<3>() =
            Eigen::Map<Eigen::Matrix<T, 3, 1>>(rotation_error_angle_axis);

        Eigen::Map<Eigen::Matrix<T, 6, 1>> weighted_residual(residuals);
        weighted_residual = sqrt_information.template cast<T>() * residual;
        return true;
    }

    Eigen::Quaterniond measurement_rotation;
    Eigen::Vector3d measurement_translation;
    Matrix6d sqrt_information;
};

}  // namespace

CeresPoseGraphOptimizer::CeresPoseGraphOptimizer(int max_iterations, double stop_delta_norm,
                                                 double damping_lambda)
    : max_iterations_(max_iterations),
      stop_delta_norm_(stop_delta_norm),
      damping_lambda_(damping_lambda) {}

void CeresPoseGraphOptimizer::addNode(int id, const Pose& initial_pose) {
    if (id != static_cast<int>(node_poses_.size())) {
        throw std::runtime_error("Pose graph node ids must be contiguous.");
    }
    node_poses_.push_back(initial_pose);
}

void CeresPoseGraphOptimizer::addEdge(const GraphEdge& edge) {
    edges_.push_back(edge);
}

void CeresPoseGraphOptimizer::removeLastEdge() {
    if (!edges_.empty()) {
        edges_.pop_back();
    }
}

bool CeresPoseGraphOptimizer::optimize() {
    if (node_poses_.size() < 2 || edges_.empty()) {
        return true;
    }

    std::vector<PoseParameters> parameters(node_poses_.size());
    for (std::size_t i = 0; i < node_poses_.size(); ++i) {
        parameters[i] = poseToParameters(node_poses_[i]);
    }

    ceres::Problem problem;
    auto* quaternion_manifold = new ceres::EigenQuaternionManifold();
    for (std::size_t i = 0; i < parameters.size(); ++i) {
        problem.AddParameterBlock(parameters[i].quaternion_xyzw.data(), 4, quaternion_manifold);
        problem.AddParameterBlock(parameters[i].translation_xyz.data(), 3);
    }
    problem.SetParameterBlockConstant(parameters.front().quaternion_xyzw.data());
    problem.SetParameterBlockConstant(parameters.front().translation_xyz.data());

    for (const auto& edge : edges_) {
        auto* cost_function =
            new ceres::AutoDiffCostFunction<PoseGraphEdgeCostFunctor, 6, 4, 3, 4, 3>(
                new PoseGraphEdgeCostFunctor(edge.relative_pose, edge.information));
        problem.AddResidualBlock(cost_function, nullptr, parameters.at(edge.from_id).quaternion_xyzw.data(),
                                 parameters.at(edge.from_id).translation_xyz.data(),
                                 parameters.at(edge.to_id).quaternion_xyzw.data(),
                                 parameters.at(edge.to_id).translation_xyz.data());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = max_iterations_;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.parameter_tolerance = std::max(stop_delta_norm_, 1e-12);
    options.function_tolerance = std::max(stop_delta_norm_ * 1e-2, 1e-12);
    options.gradient_tolerance = std::max(stop_delta_norm_ * 1e-4, 1e-16);
    options.min_lm_diagonal = std::max(damping_lambda_, 1e-12);
    options.logging_type = ceres::SILENT;
    options.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        return false;
    }

    for (std::size_t i = 0; i < node_poses_.size(); ++i) {
        node_poses_[i] = parametersToPose(parameters[i], node_poses_[i].time_stamp.sec());
    }
    return true;
}

const std::vector<Pose>& CeresPoseGraphOptimizer::nodePoses() const {
    return node_poses_;
}

CeresPoseGraphOptimizer::PoseParameters CeresPoseGraphOptimizer::poseToParameters(const Pose& pose) {
    PoseParameters parameters{};
    const Eigen::Quaterniond normalized_rotation = pose.rotation.normalized();
    parameters.quaternion_xyzw = {normalized_rotation.x(), normalized_rotation.y(),
                                  normalized_rotation.z(), normalized_rotation.w()};
    parameters.translation_xyz = {pose.position.x(), pose.position.y(), pose.position.z()};
    return parameters;
}

Pose CeresPoseGraphOptimizer::parametersToPose(const PoseParameters& parameters, double stamp_sec) {
    return makePose(Eigen::Quaterniond(parameters.quaternion_xyzw[3], parameters.quaternion_xyzw[0],
                                       parameters.quaternion_xyzw[1], parameters.quaternion_xyzw[2]),
                    Eigen::Vector3d(parameters.translation_xyz[0], parameters.translation_xyz[1],
                                    parameters.translation_xyz[2]),
                    stamp_sec);
}

}  // namespace IESKFSLAM
