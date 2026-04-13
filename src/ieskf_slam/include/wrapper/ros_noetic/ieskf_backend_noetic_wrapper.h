#pragma once
#include <ros/ros.h>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>
#include "ieskf_slam/CloudWithPose.h"
#include "ieskf_slam/modules/backend/backend_coordinator.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include <nav_msgs/Path.h>

namespace ROSNoetic
{
    class IESKFBackEndWrapper
    {
    private:
        IESKFSLAM::BackendCoordinator backend_coordinator;
        ros::Subscriber cloud_with_pose_sub;
        ros::Publisher optimized_map_pub;
        ros::Publisher optimized_path_pub;
        std::mutex backend_mutex_;
        std::mutex cloud_with_pose_queue_mutex_;
        std::mutex optimized_map_state_mutex_;
        std::mutex anomaly_log_mutex_;
        std::condition_variable cloud_with_pose_queue_cv_;
        std::condition_variable optimized_map_cv_;
        std::deque<ieskf_slam::CloudWithPose::ConstPtr> pending_cloud_with_pose_queue_;
        std::thread backend_worker_thread_;
        std::thread optimized_map_thread_;
        ros::Time latest_optimized_map_stamp_;
        double optimized_map_publish_hz_ = 1.0;
        std::uint64_t optimized_map_version_ = 0;
        std::uint64_t published_map_version_ = 0;
        std::size_t cloud_with_pose_queue_capacity_ = 100;
        ros::Time last_cloud_with_pose_stamp_;
        bool stop_backend_worker_thread_ = false;
        bool stop_optimized_map_thread_ = false;
        bool enable_record_raw_ = false;
        bool enable_record_optimized_ = false;
        bool enable_save_dense_map_ = true;
        std::string raw_record_file_name_ = "backend_raw_keyframes.txt";
        std::string optimized_record_file_name_ = "backend_optimized_keyframes.txt";
        std::string dense_map_file_name_ = "optimized_map_dense.pcd";
        std::string anomaly_log_file_name_ = "backend_anomalies.txt";
        std::ofstream anomaly_log_file_;
        void CloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPose::ConstPtr& msg);
        void backendWorkerLoop();
        void processCloudWithPoseMsg(const ieskf_slam::CloudWithPose::ConstPtr& msg);
        void publishOptimizedPath(const std::vector<IESKFSLAM::Pose>& optimized_poses, const ros::Time& stamp);
        void prepareRecordFile(const std::string& file_name) const;
        void appendPoseToRecordFile(const std::string& file_name, const IESKFSLAM::Pose& pose) const;
        void writeTrajectoryToFile(const std::string& file_name,
                                   const std::vector<IESKFSLAM::Pose>& poses) const;
        void writeAnomalyLogLine(const std::string& line);
        void saveDenseMapToFile();
        void optimizedMapPublishLoop();
    public:
        IESKFBackEndWrapper(ros::NodeHandle& nh);
        ~IESKFBackEndWrapper();
    };
      
} // namespace ROSNoetic
