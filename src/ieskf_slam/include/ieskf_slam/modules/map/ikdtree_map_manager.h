#pragma once

#include "ieskf_slam/modules/map/map_manager_base.h"
#include "ieskf_slam/third_party/ikd_tree/ikd_Tree.h"
#include <memory>

namespace IESKFSLAM {

class IKDTreeMapManager : public MapManagerBase {
public:
    using Ptr = std::shared_ptr<IKDTreeMapManager>;

    IKDTreeMapManager(const std::string& config_path, const std::string& prefix);
    ~IKDTreeMapManager() override;

    void reset() override;
    void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q,
                 const Eigen::Vector3d& pos_t) override;
    PCLPointCloudConstPtr getLocalMap() const override;
    bool nearestKSearch(const Point& query_point, int k, std::vector<Point>& nearest_points,
                        std::vector<float>& squared_distances) const override;

private:
    static ::PointType toIKDPoint(const Point& point);
    static Point fromIKDPoint(const ::PointType& point);
    std::vector<BoxPointType> buildDeleteBoxes(const Eigen::Vector3d& pos_t) const;
    void pruneLocalMap(const Eigen::Vector3d& pos_t);

    PCLPointCloudPtr local_map_ptr_;
    std::unique_ptr<KD_TREE> ikdtree_ptr_;
    float map_side_length_2_ = 500.0f;
    float map_resolution_ = 0.5f;
    float ikdtree_delete_criterion_ = 0.5f;
    float ikdtree_balance_criterion_ = 0.7f;
    float ikdtree_downsample_size_ = 0.5f;
};

}  // namespace IESKFSLAM
