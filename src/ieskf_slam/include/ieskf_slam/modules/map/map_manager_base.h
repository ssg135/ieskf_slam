#pragma once

#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

namespace IESKFSLAM {

class MapManagerBase : protected ModuleBase {
public:
    using Ptr = std::shared_ptr<MapManagerBase>;
    using ConstPtr = std::shared_ptr<const MapManagerBase>;

    virtual ~MapManagerBase() = default;
    virtual void reset() = 0;
    virtual void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q,
                         const Eigen::Vector3d& pos_t) = 0;
    virtual PCLPointCloudConstPtr getLocalMap() const = 0;
    virtual bool nearestKSearch(const Point& query_point, int k, std::vector<Point>& nearest_points,
                                std::vector<float>& squared_distances) const = 0;

protected:
    MapManagerBase(const std::string& config_path, const std::string& prefix,
                   const std::string& module_name)
        : ModuleBase(config_path, prefix, module_name) {}
};

MapManagerBase::Ptr CreateMapManager(const std::string& config_path, const std::string& prefix);

}  // namespace IESKFSLAM
