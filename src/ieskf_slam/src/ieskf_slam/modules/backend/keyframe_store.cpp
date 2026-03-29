#include "ieskf_slam/modules/backend/keyframe_store.h"
#include <stdexcept>

namespace IESKFSLAM {

int KeyframeStore::addKeyframe(Keyframe keyframe) {
    if (keyframe.id < 0) {
        keyframe.id = static_cast<int>(keyframes_.size());
    }
    if (keyframe.id != static_cast<int>(keyframes_.size())) {
        throw std::runtime_error("Keyframe ids must be contiguous.");
    }
    keyframes_.push_back(std::move(keyframe));
    return keyframes_.back().id;
}

bool KeyframeStore::empty() const {
    return keyframes_.empty();
}

std::size_t KeyframeStore::size() const {
    return keyframes_.size();
}

const Keyframe& KeyframeStore::get(int id) const {
    return keyframes_.at(id);
}

Keyframe& KeyframeStore::mutableGet(int id) {
    return keyframes_.at(id);
}

const std::vector<Keyframe>& KeyframeStore::all() const {
    return keyframes_;
}

void KeyframeStore::updateOptimizedPoses(const std::vector<Pose>& optimized_poses) {
    if (optimized_poses.size() != keyframes_.size()) {
        throw std::runtime_error("Optimized pose count does not match keyframe count.");
    }
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
        keyframes_[i].optimized_pose = optimized_poses[i];
    }
}

}  // namespace IESKFSLAM
