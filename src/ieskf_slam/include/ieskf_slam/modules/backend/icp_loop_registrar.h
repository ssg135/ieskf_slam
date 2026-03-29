#pragma once

#include "ieskf_slam/modules/backend/loop_registrar_base.h"

namespace IESKFSLAM {

class ICPLoopRegistrar : public LoopRegistrarBase {
public:
    ICPLoopRegistrar(int max_iterations, double max_correspondence_distance, double fitness_threshold,
                     double translation_information_scale, double rotation_information_scale);

    LoopConstraint registerLoop(const Keyframe& query_keyframe, const Keyframe& target_keyframe,
                                const LoopCandidate& candidate) override;

private:
    int max_iterations_;
    double max_correspondence_distance_;
    double fitness_threshold_;
    double translation_information_scale_;
    double rotation_information_scale_;
};

}  // namespace IESKFSLAM
