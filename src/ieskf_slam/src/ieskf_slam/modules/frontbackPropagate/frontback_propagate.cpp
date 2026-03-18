#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"
#include "ieskf_slam/common/logging.h"
#include <algorithm>


namespace IESKFSLAM {
    FrontbackPropagate::FrontbackPropagate(){}
    FrontbackPropagate::~FrontbackPropagate(){}
    void FrontbackPropagate::initialize(const IMU& init_imu, double init_acc_unit_scale){
        last_imu = init_imu;
        acc_unit_scale = init_acc_unit_scale;
        initialized_ = true;
    }

    FrontbackPropagate::PropagationResult FrontbackPropagate::forwardPropagate(
        const MeasureGroup& mg, IESKF::Ptr ieskf_ptr) {
        PropagationResult result;
        if (!initialized_) {
            if (mg.imus.empty()) {
                SLAM_LOG_WARN << "FrontbackPropagate received empty imu data before initialization";
                return result;
            }
            initialize(mg.imus.front(), acc_unit_scale);
        }
        std::deque<IMU> imus = mg.imus;
        imus.push_front(last_imu);
        IMU in;
        double dt{0};
        std::vector<IMUPose6d> imu_pose;
        auto imu_state = ieskf_ptr->getX();
        Timestamp zero_time;
        zero_time.fromSec(0.0);
        imu_pose.emplace_back(zero_time, last_avr_acc_nominal, last_avr_gyro_nominal, imu_state.velocity, imu_state.position, imu_state.rotation);
        for(auto it_imu = imus.begin(); it_imu < imus.end()-1; it_imu++ ){
            auto&& head = *(it_imu);
            auto&& tail = *(it_imu + 1);
            if (tail.time_stamp.sec() < last_lidar_end_time.sec()){
                continue;
            }
            if(head.time_stamp.sec() < last_lidar_end_time.sec()){
                dt = tail.time_stamp.sec() - last_lidar_end_time.sec();
            }
            else{
                dt = tail.time_stamp.sec() - head.time_stamp.sec();
            }
            auto avr_gyro =0.5*(head.gyroscope + tail.gyroscope);
            auto avr_acc =0.5*(head.acceleration + tail.acceleration)*acc_unit_scale;
            in.gyroscope = avr_gyro;
            in.acceleration = avr_acc;
            ieskf_ptr->predict(in, dt);
            imu_state = ieskf_ptr->getX();
            last_avr_gyro_nominal = avr_gyro - imu_state.bg;
            last_avr_acc_nominal = imu_state.rotation*(avr_acc - imu_state.ba) + imu_state.gravity;
            Timestamp timeoffset_from_lidar_begin;
            timeoffset_from_lidar_begin.fromSec(tail.time_stamp.sec() - mg.lidar_begin_time);
            imu_pose.emplace_back(timeoffset_from_lidar_begin, last_avr_acc_nominal, last_avr_gyro_nominal, imu_state.velocity, imu_state.position, imu_state.rotation);     
        }
        dt = mg.lidar_end_time - imus.back().time_stamp.sec();
        ieskf_ptr->predict(in, dt);
        imu_state = ieskf_ptr->getX();
        last_imu = imus.back();
        last_lidar_end_time.fromSec(mg.lidar_end_time);
        result.valid = true;
        result.final_state = imu_state;
        result.imu_poses = std::move(imu_pose);
        return result;
    }

    void FrontbackPropagate::deskewPointCloud(
        PointCloud& point_cloud, const IESKF::State18& frame_end_state,
        const std::vector<IMUPose6d>& imu_poses) const {
        auto& points = point_cloud.cloud_ptr->points;
        if (points.empty() || imu_poses.size() < 2) {
            return;
        }
        std::sort(points.begin(), points.end(), [](const Point& x, const Point& y) {
            return x.offset_time < y.offset_time;
        });
        auto point_in_cloud = points.end() - 1;
        for (auto it_imu_pose = imu_poses.end() - 1; it_imu_pose != imu_poses.begin(); it_imu_pose--){
            auto&& head = *(it_imu_pose-1);
            auto&& tail = *it_imu_pose;
            const Eigen::Matrix3d R_begin = head.rot.toRotationMatrix();
            const Eigen::Vector3d vel_begin = head.vel;
            const Eigen::Vector3d pos_begin = head.pos;
            const Eigen::Vector3d angvel_head_to_tail = tail.angvel;
            const Eigen::Vector3d acc_head_to_tail = tail.acc;
            for(; point_in_cloud->offset_time/1e9 > head.time.sec(); --point_in_cloud){
                double dt = point_in_cloud->offset_time/1e9 - head.time.sec();
                Eigen::Matrix3d R_i = R_begin * so3Exp(angvel_head_to_tail*dt);
                Eigen::Vector3d P_i = {point_in_cloud->x, point_in_cloud->y, point_in_cloud->z};
                Eigen::Vector3d T_ie = pos_begin + vel_begin*dt + acc_head_to_tail * dt*dt /2 - frame_end_state.position;
                Eigen::Vector3d P_compensate = frame_end_state.rotation.conjugate() * (R_i * P_i + T_ie);
                point_in_cloud->x = P_compensate(0);
                point_in_cloud->y = P_compensate(1);
                point_in_cloud->z = P_compensate(2);
                if(point_in_cloud == points.begin()){
                    break;
                }
            }
        }
    }
}
