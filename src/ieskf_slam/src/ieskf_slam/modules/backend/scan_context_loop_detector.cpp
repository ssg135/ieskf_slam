#include "ieskf_slam/modules/backend/scan_context_loop_detector.h"

namespace IESKFSLAM {

pcl::PointCloud<SCPointType> ScanContextLoopDetector::toScanContextCloud(const PCLPointCloud& cloud) {
    pcl::PointCloud<SCPointType> scan_context_cloud;
    scan_context_cloud.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        SCPointType sc_point;
        sc_point.x = point.x;
        sc_point.y = point.y;
        sc_point.z = point.z;
        sc_point.intensity = point.intensity;
        scan_context_cloud.push_back(sc_point);
    }
    return scan_context_cloud;
}

void ScanContextLoopDetector::addKeyframe(const Keyframe& keyframe) {
    auto scan_context_cloud = toScanContextCloud(keyframe.downsampled_cloud);
    sc_manager_.makeAndSaveScancontextAndKeys(scan_context_cloud);
    keyframe_ids_.push_back(keyframe.id);
}

LoopCandidate ScanContextLoopDetector::detect(const Keyframe& query_keyframe) {
    LoopCandidate candidate;
    candidate.query_id = query_keyframe.id;

    if (keyframe_ids_.empty() || keyframe_ids_.back() != query_keyframe.id) {
        return candidate;
    }

    const auto detection_result = sc_manager_.detectLoopClosureID();
    if (detection_result.first < 0 ||
        detection_result.first >= static_cast<int>(keyframe_ids_.size())) {
        return candidate;
    }

    candidate.target_id = keyframe_ids_[detection_result.first];
    candidate.yaw_init_rad = detection_result.second;
    candidate.valid = true;
    return candidate;
}

}  // namespace IESKFSLAM
