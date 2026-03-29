#pragma once

#include "ieskf_slam/modules/backend/pose_graph_optimizer_base.h"
#include <vector>

namespace IESKFSLAM {

class SimplePoseGraphOptimizer : public PoseGraphOptimizerBase {
public:
    SimplePoseGraphOptimizer(int max_iterations, double stop_delta_norm, double damping_lambda);

    void addNode(int id, const Pose& initial_pose) override;
    void addEdge(const GraphEdge& edge) override;
    void removeLastEdge() override;
    bool optimize() override;
    const std::vector<Pose>& nodePoses() const override;

private:
    Vector6d edgeResidual(const GraphEdge& edge, const std::vector<Pose>& poses) const;
    Matrix6d numericJacobian(const GraphEdge& edge, const std::vector<Pose>& poses, bool perturb_from) const;

    int max_iterations_;
    double stop_delta_norm_;
    double damping_lambda_;
    std::vector<Pose> node_poses_;
    std::vector<GraphEdge> edges_;
};

}  // namespace IESKFSLAM
