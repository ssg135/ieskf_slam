#pragma once

#include "ieskf_slam/modules/backend/backend_types.h"
#include <vector>

namespace IESKFSLAM {

class KeyframeStore {
public:
    int addKeyframe(Keyframe keyframe);
    bool empty() const;
    std::size_t size() const;
    const Keyframe& get(int id) const;
    Keyframe& mutableGet(int id);
    const std::vector<Keyframe>& all() const;
    void updateOptimizedPoses(const std::vector<Pose>& optimized_poses);

private:
    std::vector<Keyframe> keyframes_;
};

}  // namespace IESKFSLAM
