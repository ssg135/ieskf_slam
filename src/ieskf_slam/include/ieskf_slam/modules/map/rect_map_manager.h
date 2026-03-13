#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>
#include "ieskf_slam/type/base_type.h"
namespace IESKFSLAM{
    class RectMapManager : private ModuleBase{
        private:
            PCLPointCloudPtr local_map_ptr;
            KdTreePtr kdtree_ptr;
        public:
        using Ptr = std::shared_ptr<RectMapManager>;
            RectMapManager(const std::string&config_path,const std::string&prefix);
            ~RectMapManager();
            void reset();
            void addScan(const PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q, const Eigen::Vector3d pos_t);
            PCLPointCloudConstPtr getLocalMap();
            KdTreeConstPtr readKdtree();
    };
}