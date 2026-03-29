#pragma once

#include "ieskf_slam/modules/backend/backend_types.h"

namespace IESKFSLAM {

class LoopDetectorBase {
public:
    virtual ~LoopDetectorBase() = default;
    virtual void addKeyframe(const Keyframe& keyframe) = 0;
    virtual LoopCandidate detect(const Keyframe& query_keyframe) = 0;
};

}  // namespace IESKFSLAM
