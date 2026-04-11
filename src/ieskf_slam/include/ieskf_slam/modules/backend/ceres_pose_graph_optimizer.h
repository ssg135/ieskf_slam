#pragma once

#include "ieskf_slam/modules/backend/pose_graph_optimizer_base.h"
#include <array>
#include <vector>

namespace IESKFSLAM {

class CeresPoseGraphOptimizer : public PoseGraphOptimizerBase {
public:
    CeresPoseGraphOptimizer(int max_iterations, double stop_delta_norm, double damping_lambda);

    void addNode(int id, const Pose& initial_pose) override;
    void addEdge(const GraphEdge& edge) override;
    void removeLastEdge() override;
    bool optimize() override;
    const std::vector<Pose>& nodePoses() const override;

private:
    struct PoseParameters {
        std::array<double, 4> quaternion_xyzw;
        std::array<double, 3> translation_xyz;
    };

    static PoseParameters poseToParameters(const Pose& pose);
    static Pose parametersToPose(const PoseParameters& parameters, double stamp_sec);

    int max_iterations_;
    double stop_delta_norm_;
    double damping_lambda_;
    std::vector<Pose> node_poses_;
    std::vector<GraphEdge> edges_;
};

}  // namespace IESKFSLAM
