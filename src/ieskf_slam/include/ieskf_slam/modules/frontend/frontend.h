#pragma once
#include <deque>
#include <memory>
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/modules/module_base.h"

namespace IESKFSLAM{
    class FrontEnd: private ModuleBase{
        public: using Ptr = std::shared_ptr<FrontEnd>;
        private:
        std::deque<IMU> imu_deque;
        std::deque<PointCloud> pointcloud_deque;
        std::deque<Pose> pose_deque;
        PCLPointCloud current_pointcloud;
        public:
        FrontEnd(const std::string&config_path, const std::string&prefix);
        ~FrontEnd();
        void addImu(const IMU& imu);
        void addPointCloud(const PointCloud& pointcloud);
        void addPose(const Pose& pose);
        bool track();
        const PCLPointCloud& readCurrentPointCloud() const;
    };
}
