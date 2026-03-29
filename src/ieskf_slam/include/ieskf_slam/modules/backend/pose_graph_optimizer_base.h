#pragma once

#include "ieskf_slam/modules/backend/backend_types.h"
#include <vector>

namespace IESKFSLAM {

class PoseGraphOptimizerBase {
public:
    virtual ~PoseGraphOptimizerBase() = default;
    virtual void addNode(int id, const Pose& initial_pose) = 0;
    virtual void addEdge(const GraphEdge& edge) = 0;
    virtual void removeLastEdge() = 0;
    virtual bool optimize() = 0;
    virtual const std::vector<Pose>& nodePoses() const = 0;
};

}  // namespace IESKFSLAM
