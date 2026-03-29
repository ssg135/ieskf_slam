#pragma once

#include "ieskf_slam/modules/backend/backend_types.h"

namespace IESKFSLAM {

class LoopRegistrarBase {
public:
    virtual ~LoopRegistrarBase() = default;
    virtual LoopConstraint registerLoop(const Keyframe& query_keyframe, const Keyframe& target_keyframe,
                                        const LoopCandidate& candidate) = 0;
};

}  // namespace IESKFSLAM
