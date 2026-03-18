#pragma once

#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/measure_group.h"
#include <memory>
#include <vector>
#include "ieskf_slam/math/SO3.h"

namespace IESKFSLAM {
    class FrontbackPropagate{
        private:
            IMU last_imu;
            double acc_unit_scale = 1.0;
            bool initialized_ = false;
            Timestamp last_lidar_end_time;
            Eigen::Vector3d last_avr_acc_nominal = Eigen::Vector3d::Zero();
            Eigen::Vector3d last_avr_gyro_nominal = Eigen::Vector3d::Zero();
        public:
            struct IMUPose6d
            {
                IESKFSLAM::Timestamp time;                     
                Eigen::Vector3d acc = Eigen::Vector3d::Zero();    
                Eigen::Vector3d angvel = Eigen::Vector3d::Zero(); 
                Eigen::Vector3d vel = Eigen::Vector3d::Zero();    
                Eigen::Vector3d pos = Eigen::Vector3d::Zero();     
                Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); 
                IMUPose6d() = default;
                // 构造函数，支持部分初始化
                IMUPose6d(const IESKFSLAM::Timestamp& t,
                        const Eigen::Vector3d& a = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& av = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& v = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& p = Eigen::Vector3d::Zero(),
                        const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity())
                    : time(t), acc(a), angvel(av), vel(v), pos(p), rot(q) {}
            };
            struct PropagationResult {
                bool valid = false;
                IESKF::State18 final_state;
                std::vector<IMUPose6d> imu_poses;
            };
            using Ptr = std::shared_ptr<FrontbackPropagate>;
            FrontbackPropagate();
            ~FrontbackPropagate();
            void initialize(const IMU& init_imu, double init_acc_unit_scale);
            PropagationResult forwardPropagate(const MeasureGroup& mg, IESKF::Ptr ieskf_ptr);
            void deskewPointCloud(PointCloud& point_cloud, const IESKF::State18& frame_end_state,
                                  const std::vector<IMUPose6d>& imu_poses) const;
    }; 
}
