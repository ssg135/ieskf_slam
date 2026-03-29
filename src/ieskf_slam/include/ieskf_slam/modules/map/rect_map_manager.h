#pragma once
#include "ieskf_slam/modules/map/map_manager_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>
#include "ieskf_slam/type/base_type.h"
namespace IESKFSLAM{
    class RectMapManager : public MapManagerBase{
        private:
            PCLPointCloudPtr local_map_ptr;
            KdTreePtr kdtree_ptr;
            float map_side_length_2;
            float map_resolution;
        public:
        using Ptr = std::shared_ptr<RectMapManager>;
            RectMapManager(const std::string&config_path,const std::string&prefix);
            ~RectMapManager() override;
            void reset() override;
            void addScan(const PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q,
                         const Eigen::Vector3d& pos_t) override;
            PCLPointCloudConstPtr getLocalMap() const override;
            bool nearestKSearch(const Point& query_point, int k, std::vector<Point>& nearest_points,
                                std::vector<float>& squared_distances) const override;
    };
}
