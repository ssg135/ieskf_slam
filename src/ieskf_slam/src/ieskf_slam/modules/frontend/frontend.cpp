#include "ieskf_slam/modules/frontend/frontend.h"
#include "ieskf_slam/common/logging.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/math/math.h"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <memory>
#include <pcl/common/transforms.h>
#include <ieskf_slam/globaldefine.h>

namespace IESKFSLAM{
    FrontEnd::FrontEnd(const std::string&config_path, const std::string&prefix):ModuleBase(config_path, prefix, "FrontEnd"){
        float leaf_size;
        readParam("filter_leaf_size",leaf_size,0.5f);
        voxel_filter.setLeafSize(leaf_size,leaf_size,leaf_size);
        std::vector<double>extrin_v;
        readParam("extrin_r",extrin_v,std::vector<double>());
        extrin_r.setIdentity();
        extrin_t.setZero();
        if(extrin_v.size() == 9){
            Eigen::Matrix3d extrin_r33;
            extrin_r33<<extrin_v[0],extrin_v[1],extrin_v[2],extrin_v[3],extrin_v[4],extrin_v[5],extrin_v[6],extrin_v[7],extrin_v[8];
            extrin_r = extrin_r33;
        }else if (extrin_v.size() == 4){
            extrin_r.x() = extrin_v[0];
            extrin_r.y() = extrin_v[1];
            extrin_r.z() = extrin_v[2];
            extrin_r.w() = extrin_v[3];
        }
        readParam("extrin_t",extrin_v,std::vector<double>());
        readParam("enable_record", enable_record, false);
        readParam("record_file_name", record_file_name, std::string("default.txt"));
        if (enable_record) {
            record_file.open(RESULT_DIR + record_file_name, std::ios::out | std::ios::app);
        }
        print_table();
        if(extrin_v.size()==3){
            extrin_t<<extrin_v[0],extrin_v[1],extrin_v[2];
        }
        SLAM_LOG_INFO << "FrontEnd initialized with filter_leaf_size=" << leaf_size
                      << ", extrin_r_param_size=" << extrin_v.size()
                      << ", extrin_t=[" << extrin_t.transpose() << "]";
        ieskf_ptr = std::make_shared<IESKF>(config_path, "ieskf");
        map_ptr = std::make_shared<RectMapManager>(config_path, "map");
        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(filter_point_cloud_ptr, map_ptr->getLocalMap(), map_ptr->readKdtree());
    }
    FrontEnd::~FrontEnd(){record_file.close();}
    void FrontEnd::addImu(const IMU& imu){
        imu_deque.push_back(imu);
    }
    void FrontEnd::addPointCloud(const PointCloud& pointcloud){
        pointcloud_deque.push_back(pointcloud);
        pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr,
                                 *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
    }
    void FrontEnd::addPose(const Pose& pose){
        pose_deque.push_back(pose);
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud() const{
        return *filter_point_cloud_ptr;
    }
    const PCLPointCloud& FrontEnd::readCurrentLocalMap() const{
        return *map_ptr->getLocalMap();
    }
    bool FrontEnd::track(){
        const auto track_begin = std::chrono::steady_clock::now();
        MeasureGroup mg;
        if(syncMeasureGroup(mg)){
            const auto sync_end = std::chrono::steady_clock::now();
            if (!imu_inited) {
                map_ptr->reset();
                map_ptr->addScan(mg.point_cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
                initState(mg);
                const auto init_end = std::chrono::steady_clock::now();
                LOG_EVERY_N(WARNING, 20) << "track timing(init) total_ms="
                                         << std::chrono::duration<double, std::milli>(init_end - track_begin).count()
                                         << ", sync_ms="
                                         << std::chrono::duration<double, std::milli>(sync_end - track_begin).count()
                                         << ", init_ms="
                                         << std::chrono::duration<double, std::milli>(init_end - sync_end).count();
                SLAM_LOG_INFO << "FrontEnd IMU initialized with " << mg.imus.size() << " imu samples";
                return false;
            }
            const auto propagation = fbpropagate_ptr->forwardPropagate(mg, ieskf_ptr);
            const auto propagation_end = std::chrono::steady_clock::now();
            if (!propagation.valid) {
                SLAM_LOG_WARN << "FrontbackPropagate forward propagation failed";
                return false;
            }
            ieskf_ptr->setX(propagation.final_state);
            fbpropagate_ptr->deskewPointCloud(mg.point_cloud, propagation.final_state, propagation.imu_poses);
            voxel_filter.setInputCloud(mg.point_cloud.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);
            const auto filter_end = std::chrono::steady_clock::now();
            if (filter_point_cloud_ptr->empty()) {
                LOG_EVERY_N(WARNING, 20) << "Filtered point cloud is empty after deskew, raw cloud size="
                                         << mg.point_cloud.cloud_ptr->size();
                return false;
            }
            if (!ieskf_ptr->update()) {
                SLAM_LOG_WARN << "IESKF update did not converge";
            }
            const auto update_end = std::chrono::steady_clock::now();
            auto x = ieskf_ptr->getX();
            if (enable_record) {
                record_file << std::setprecision(15) << mg.lidar_end_time << " "
                << x.position.x() << " " << x.position.y() << " "
                << x.position.z() << " " << x.rotation.x() << " "
                << x.rotation.y() << " " << x.rotation.z() << " "
                << x.rotation.w() << std::endl;
}
            map_ptr->addScan(filter_point_cloud_ptr,x.rotation,x.position);
            const auto map_end = std::chrono::steady_clock::now();
            LOG_EVERY_N(WARNING, 20) << "track timing total_ms="
                                     << std::chrono::duration<double, std::milli>(map_end - track_begin).count()
                                     << ", sync_ms="
                                     << std::chrono::duration<double, std::milli>(sync_end - track_begin).count()
                                     << ", propagate_ms="
                                     << std::chrono::duration<double, std::milli>(propagation_end - sync_end).count()
                                     << ", deskew_filter_ms="
                                     << std::chrono::duration<double, std::milli>(filter_end - propagation_end).count()
                                     << ", update_ms="
                                     << std::chrono::duration<double, std::milli>(update_end - filter_end).count()
                                     << ", map_ms="
                                     << std::chrono::duration<double, std::milli>(map_end - update_end).count()
                                     << ", raw_points=" << mg.point_cloud.cloud_ptr->size()
                                     << ", filtered_points=" << filter_point_cloud_ptr->size();
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
        if(pointcloud_deque.front().cloud_ptr->empty()){
            LOG_EVERY_N(WARNING, 20) << "Dropping empty lidar frame";
            pointcloud_deque.pop_front();
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
            LOG_EVERY_N(WARNING, 20) << "Dropping stale lidar frame. cloud_end_time=" << cloud_end_time
                                     << ", imu_start_time=" << imu_start_time;
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
            LOG_EVERY_N(WARNING, 20) << "Insufficient imu samples for lidar frame: " << mg.imus.size()
                                     << ", lidar_window=[" << mg.lidar_begin_time << ", "
                                     << mg.lidar_end_time << "]";
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
        const double acc_unit_scale = GRAVITY / mean_acc.norm();
        x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
        fbpropagate_ptr->initialize(mg.imus.back(), acc_unit_scale);
        ieskf_ptr->setX(x);
        imu_inited = true;
        SLAM_LOG_INFO << "Initial state set. acc_unit_scale=" << acc_unit_scale
                      << ", bg=[" << x.bg.transpose() << "]"
                      << ", gravity=[" << x.gravity.transpose() << "]";
    }
    const IESKF::State18& FrontEnd::readState()const{
        return ieskf_ptr->getX();
    }

}
