#include "ieskf_slam/modules/frontend/frontend.h"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <pcl/common/transforms.h>

namespace IESKFSLAM{
    FrontEnd::FrontEnd(const std::string&config_path, const std::string&prefix):ModuleBase(config_path, prefix, "FrontEnd"){
        ieskf_ptr = std::make_shared<IESKF>(config_path, "ieskf");
        map_ptr = std::make_shared<RectMapManager>(config_path, "map");
        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
    }
    FrontEnd::~FrontEnd(){}
    void FrontEnd::addImu(const IMU& imu){
        imu_deque.push_back(imu);
    }
    void FrontEnd::addPointCloud(const PointCloud& pointcloud){
        pointcloud_deque.push_back(pointcloud);
    }
    void FrontEnd::addPose(const Pose& pose){
        pose_deque.push_back(pose);
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud() const{
        return current_pointcloud;
    }
    bool FrontEnd::track(){
        MeasureGroup mg;
        if(syncMeasureGroup(mg)){
            if (!imu_inited) {
                map_ptr->reset();
                map_ptr->addScan(mg.point_cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            fbpropagate_ptr->propagate(mg, ieskf_ptr);
            //std::cout<<mg.imus.size()<<" scale: "<<imu_scale<<std::endl;
            return true;
        }
        return false;
    }
    bool FrontEnd::syncMeasureGroup(MeasureGroup& mg){
        mg.imus.clear();
        mg.point_cloud.cloud_ptr->clear();
        if(pointcloud_deque.empty() || imu_deque.empty()){
            return false;
        }
        double cloud_strat_time = pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque.front().cloud_ptr->back().offset_time/1e9 + cloud_strat_time;
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double imu_end_time = imu_deque.back().time_stamp.sec();
        if(imu_end_time < cloud_end_time){
            return false;
        }
        if(cloud_end_time < imu_start_time){
            pointcloud_deque.pop_front();
            return false;
        }
        mg.point_cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_strat_time;
        mg.lidar_end_time = cloud_end_time;
        while(!imu_deque.empty()){
            if(imu_deque.front().time_stamp.sec() < cloud_end_time){
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
            }
            else{
                break;
            }
        }   
        if(mg.imus.size() <= 5){
            return false;
        }
        else {
            return true;
        }

    }
    void FrontEnd::initState(MeasureGroup &mg){
        Eigen::Vector3d sum_acc {0,0,0};
        Eigen::Vector3d sum_gyro {0,0,0};
        if(imu_inited){
            return;
        }
        for(const auto& imu:mg.imus){
            sum_acc += imu.acceleration;
            sum_gyro += imu.gyroscope;
        }
        auto x = ieskf_ptr->getX();
        x.bg = sum_gyro / double(mg.imus.size());
        Eigen::Vector3d mean_acc = sum_acc / double(mg.imus.size());
        imu_scale = GRAVITY / mean_acc.norm();
        x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
        fbpropagate_ptr->imu_scale = imu_scale;
        fbpropagate_ptr->last_imu = mg.imus.back();
        ieskf_ptr->setX(x);
        imu_inited = true;
    }
    const IESKF::State18& FrontEnd::readState()const{
        return ieskf_ptr->getX();
    }

}