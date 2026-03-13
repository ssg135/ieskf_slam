#pragma once
#include <deque>
#include <memory>
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/measure_group.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"
#include "ieskf_slam/modules/frontend/lio_zh_model.h"

namespace IESKFSLAM{
    class FrontEnd: private ModuleBase{
        public: using Ptr = std::shared_ptr<FrontEnd>;
        private:
        std::deque<IMU> imu_deque;
        std::deque<PointCloud> pointcloud_deque;
        std::deque<Pose> pose_deque;
        IESKF::Ptr ieskf_ptr;
        RectMapManager::Ptr map_ptr;
        FrontbackPropagate::Ptr fbpropagate_ptr;
        LIOZHModel::Ptr lio_zh_model_ptr;
        VoxelFilter voxel_filter;
        PCLPointCloudPtr filter_point_cloud_ptr;
        bool imu_inited =false;
        double imu_scale = 1;
        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;
        public:
        FrontEnd(const std::string&config_path, const std::string&prefix);
        ~FrontEnd();
        void addImu(const IMU& imu);
        void addPointCloud(const PointCloud& pointcloud);
        void addPose(const Pose& pose);
        bool syncMeasureGroup(MeasureGroup& mg);
        void initState(MeasureGroup& mg);
        bool track();
        const PCLPointCloud& readCurrentPointCloud() const;
        const IESKF::State18& readState() const;
    };
}
