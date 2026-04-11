#pragma once

#include "ieskf_slam/common/scan_context/Scancontext.h"
#include "ieskf_slam/modules/backend/loop_detector_base.h"
#include <vector>

namespace IESKFSLAM {

class ScanContextLoopDetector : public LoopDetectorBase {
public:
    void setDistanceThreshold(double distance_threshold);
    void addKeyframe(const Keyframe& keyframe) override;
    LoopCandidate detect(const Keyframe& query_keyframe) override;

private:
    static pcl::PointCloud<SCPointType> toScanContextCloud(const PCLPointCloud& cloud);

    SCManager sc_manager_;
    std::vector<int> keyframe_ids_;
};

}  // namespace IESKFSLAM
