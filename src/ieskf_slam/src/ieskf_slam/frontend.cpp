#include "ieskf_slam/modules/frontend/frontend.h"
#include <pcl/common/transforms.h>

namespace IESKFSLAM{
    FrontEnd::FrontEnd(const std::string&config_path, const std::string&prefix):ModuleBase(config_path, prefix, "FrontEnd"){}
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
        if(pose_deque.empty() || pointcloud_deque.empty()){
            return false;
        }
        while(! pose_deque.empty() && pose_deque.front().time_stamp.nsec() < pointcloud_deque.front().time_stamp.nsec()){
            pose_deque.pop_front();
        }
        if(pose_deque.empty()){
            return false;
        }
        while(! pointcloud_deque.empty() && pointcloud_deque.front().time_stamp.nsec() < pose_deque.front().time_stamp.nsec()){
            pointcloud_deque.pop_front();
        }
        if(pointcloud_deque.empty()){
            return false;
        }
        VoxelFilter vf;
        vf.setLeafSize(0.5, 0.5, 0.5);
        vf.setInputCloud(pointcloud_deque.front().cloud_ptr);
        vf.filter(*pointcloud_deque.front().cloud_ptr);
        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = pose_deque.front().rotation.toRotationMatrix().cast<float>();
        trans.block<3,1>(0,3) = pose_deque.front().position.cast<float>();
        pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr, current_pointcloud, trans);
        pose_deque.pop_front();
        pointcloud_deque.pop_front();
        return true;
    }

}