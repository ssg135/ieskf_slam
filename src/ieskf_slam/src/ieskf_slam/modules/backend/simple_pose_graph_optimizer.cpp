#include "ieskf_slam/modules/backend/simple_pose_graph_optimizer.h"
#include "ieskf_slam/modules/backend/backend_utils.h"
#include <algorithm>
#include <stdexcept>

namespace IESKFSLAM {

namespace {

constexpr double kNumericEpsilon = 1e-6;

}  // namespace

SimplePoseGraphOptimizer::SimplePoseGraphOptimizer(int max_iterations, double stop_delta_norm,
                                                   double damping_lambda)
    : max_iterations_(max_iterations),
      stop_delta_norm_(stop_delta_norm),
      damping_lambda_(damping_lambda) {}

void SimplePoseGraphOptimizer::addNode(int id, const Pose& initial_pose) {
    if (id != static_cast<int>(node_poses_.size())) {
        throw std::runtime_error("Pose graph node ids must be contiguous.");
    }
    node_poses_.push_back(initial_pose);
}

void SimplePoseGraphOptimizer::addEdge(const GraphEdge& edge) {
    edges_.push_back(edge);
}

void SimplePoseGraphOptimizer::removeLastEdge() {
    if (!edges_.empty()) {
        edges_.pop_back();
    }
}

Vector6d SimplePoseGraphOptimizer::edgeResidual(const GraphEdge& edge,
                                                const std::vector<Pose>& poses) const {
    const Pose prediction = relativePose(poses.at(edge.from_id), poses.at(edge.to_id));
    return poseResidual(edge.relative_pose, prediction);
}

Matrix6d SimplePoseGraphOptimizer::numericJacobian(const GraphEdge& edge,
                                                   const std::vector<Pose>& poses,
                                                   bool perturb_from) const {
    Matrix6d jacobian = Matrix6d::Zero();
    const Vector6d baseline = edgeResidual(edge, poses);
    for (int col = 0; col < 6; ++col) {
        std::vector<Pose> perturbed_poses = poses;
        Vector6d delta = Vector6d::Zero();
        delta(col) = kNumericEpsilon;
        const int pose_id = perturb_from ? edge.from_id : edge.to_id;
        perturbed_poses[pose_id] = applyPoseIncrement(perturbed_poses[pose_id], delta);
        jacobian.col(col) = (edgeResidual(edge, perturbed_poses) - baseline) / kNumericEpsilon;
    }
    return jacobian;
}

bool SimplePoseGraphOptimizer::optimize() {
    if (node_poses_.size() < 2 || edges_.empty()) {
        return true;
    }

    const int variable_block_count = static_cast<int>(node_poses_.size()) - 1;
    if (variable_block_count <= 0) {
        return true;
    }
    const int variable_dim = variable_block_count * 6;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variable_dim, variable_dim);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variable_dim);

        for (const auto& edge : edges_) {
            const Vector6d residual = edgeResidual(edge, node_poses_);
            const Matrix6d jacobian_from = numericJacobian(edge, node_poses_, true);
            const Matrix6d jacobian_to = numericJacobian(edge, node_poses_, false);
            const Matrix6d weighted_information = edge.information;

            const int from_offset = edge.from_id == 0 ? -1 : (edge.from_id - 1) * 6;
            const int to_offset = edge.to_id == 0 ? -1 : (edge.to_id - 1) * 6;

            if (from_offset >= 0) {
                hessian.block<6, 6>(from_offset, from_offset) +=
                    jacobian_from.transpose() * weighted_information * jacobian_from;
                gradient.segment<6>(from_offset) +=
                    jacobian_from.transpose() * weighted_information * residual;
            }
            if (to_offset >= 0) {
                hessian.block<6, 6>(to_offset, to_offset) +=
                    jacobian_to.transpose() * weighted_information * jacobian_to;
                gradient.segment<6>(to_offset) +=
                    jacobian_to.transpose() * weighted_information * residual;
            }
            if (from_offset >= 0 && to_offset >= 0) {
                const Eigen::Matrix<double, 6, 6> cross_term =
                    jacobian_from.transpose() * weighted_information * jacobian_to;
                hessian.block<6, 6>(from_offset, to_offset) += cross_term;
                hessian.block<6, 6>(to_offset, from_offset) += cross_term.transpose();
            }
        }

        hessian.diagonal().array() += damping_lambda_;
        const Eigen::VectorXd delta = -hessian.ldlt().solve(gradient);
        if (!delta.allFinite()) {
            return false;
        }
        if (delta.norm() < stop_delta_norm_) {
            return true;
        }

        for (int node_id = 1; node_id < static_cast<int>(node_poses_.size()); ++node_id) {
            const Vector6d increment = delta.segment<6>((node_id - 1) * 6);
            node_poses_[node_id] = applyPoseIncrement(node_poses_[node_id], increment);
        }
    }
    return true;
}

const std::vector<Pose>& SimplePoseGraphOptimizer::nodePoses() const {
    return node_poses_;
}

}  // namespace IESKFSLAM
