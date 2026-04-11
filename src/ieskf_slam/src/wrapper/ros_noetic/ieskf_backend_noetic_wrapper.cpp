#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>

namespace ROSNoetic
{
    IESKFBackEndWrapper::IESKFBackEndWrapper(ros::NodeHandle& nh)
        : backend_coordinator([&nh]() {
              std::string config_file_name;
              nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
              return config_file_name.empty() ? std::string("") : CONFIG_DIR + config_file_name;
          }(),
          "back_end")
    {
        cloud_with_pose_sub = nh.subscribe("cloud_with_pose", 100, &IESKFBackEndWrapper::CloudWithPoseMsgCallBack, this);
        optimized_map_pub = nh.advertise<sensor_msgs::PointCloud2>("backend/optimized_map", 1, true);
        optimized_path_pub = nh.advertise<nav_msgs::Path>("backend/optimized_path", 1, true);
        nh.param<double>("back_end/optimized_map_publish_hz", optimized_map_publish_hz_, 1.0);
        nh.param<bool>("back_end/enable_record_raw", enable_record_raw_, false);
        nh.param<bool>("back_end/enable_record_optimized", enable_record_optimized_, false);
        nh.param<bool>("back_end/enable_save_dense_map", enable_save_dense_map_, true);
        nh.param<std::string>("back_end/raw_record_file_name", raw_record_file_name_,
                              "backend_raw_keyframes.txt");
        nh.param<std::string>("back_end/optimized_record_file_name", optimized_record_file_name_,
                              "backend_optimized_keyframes.txt");
        nh.param<std::string>("back_end/dense_map_file_name", dense_map_file_name_,
                              "optimized_map_dense.pcd");
        optimized_map_publish_hz_ = std::max(optimized_map_publish_hz_, 0.1);
        optimized_map_thread_ = std::thread(&IESKFBackEndWrapper::optimizedMapPublishLoop, this);
    }

    IESKFBackEndWrapper::~IESKFBackEndWrapper()
    {
        {
            std::lock_guard<std::mutex> lock(optimized_map_state_mutex_);
            stop_optimized_map_thread_ = true;
        }
        optimized_map_cv_.notify_all();
        if (optimized_map_thread_.joinable()) {
            optimized_map_thread_.join();
        }
        saveDenseMapToFile();
    }

    void IESKFBackEndWrapper::CloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPose::ConstPtr& msg)
    {
        IESKFSLAM::PCLPointCloud in_cloud;
        IESKFSLAM::Pose pose;
        pose.position.x() = msg->pose.position.x;
        pose.position.y() = msg->pose.position.y;
        pose.position.z() = msg->pose.position.z;
        pose.rotation.w() = msg->pose.orientation.w;
        pose.rotation.x() = msg->pose.orientation.x;
        pose.rotation.y() = msg->pose.orientation.y;
        pose.rotation.z() = msg->pose.orientation.z;
        pose.time_stamp.fromNSec(msg->point_cloud.header.stamp.toNSec());
        pcl::fromROSMsg(msg->point_cloud, in_cloud);
        if (in_cloud.empty()) {
            return;
        }
        IESKFSLAM::BackendProcessResult result;
        std::vector<IESKFSLAM::Pose> raw_poses;
        std::vector<IESKFSLAM::Pose> optimized_poses;
        {
            std::lock_guard<std::mutex> lock(backend_mutex_);
            result = backend_coordinator.processFrame(in_cloud, pose, msg->point_cloud.header.stamp.toSec());
            if (result.inserted_keyframe) {
                raw_poses = backend_coordinator.readRawPoses();
                optimized_poses = backend_coordinator.readOptimizedPoses();
            }
        }
        if (result.inserted_keyframe) {
            publishOptimizedPath(optimized_poses, msg->point_cloud.header.stamp);
            if (enable_record_raw_) {
                writeTrajectoryToFile(raw_record_file_name_, raw_poses);
            }
            if (enable_record_optimized_) {
                writeTrajectoryToFile(optimized_record_file_name_, optimized_poses);
            }
            {
                std::lock_guard<std::mutex> lock(optimized_map_state_mutex_);
                latest_optimized_map_stamp_ = msg->point_cloud.header.stamp;
                ++optimized_map_version_;
            }
            optimized_map_cv_.notify_one();
        }
    }

    void IESKFBackEndWrapper::publishOptimizedPath(const std::vector<IESKFSLAM::Pose>& optimized_poses,
                                                   const ros::Time& stamp)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";
        for (const auto& pose : optimized_poses) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = pose.position.x();
            pose_stamped.pose.position.y = pose.position.y();
            pose_stamped.pose.position.z = pose.position.z();
            pose_stamped.pose.orientation.w = pose.rotation.w();
            pose_stamped.pose.orientation.x = pose.rotation.x();
            pose_stamped.pose.orientation.y = pose.rotation.y();
            pose_stamped.pose.orientation.z = pose.rotation.z();
            path_msg.poses.push_back(pose_stamped);
        }
        optimized_path_pub.publish(path_msg);
    }

    void IESKFBackEndWrapper::writeTrajectoryToFile(const std::string& file_name,
                                                    const std::vector<IESKFSLAM::Pose>& poses) const
    {
        std::ofstream record_file(RESULT_DIR + file_name, std::ios::out | std::ios::trunc);
        if (!record_file.is_open()) {
            ROS_WARN_STREAM("failed to open backend record file: " << RESULT_DIR + file_name);
            return;
        }
        record_file << std::setprecision(15);
        for (const auto& pose : poses) {
            record_file << pose.time_stamp.sec() << " "
                        << pose.position.x() << " " << pose.position.y() << " "
                        << pose.position.z() << " " << pose.rotation.x() << " "
                        << pose.rotation.y() << " " << pose.rotation.z() << " "
                        << pose.rotation.w() << std::endl;
        }
    }

    void IESKFBackEndWrapper::saveDenseMapToFile()
    {
        if (!enable_save_dense_map_) {
            return;
        }

        IESKFSLAM::PCLPointCloud dense_map;
        {
            std::lock_guard<std::mutex> lock(backend_mutex_);
            dense_map = backend_coordinator.buildDenseOptimizedMap();
        }

        if (dense_map.empty()) {
            ROS_WARN_STREAM("dense optimized map is empty, skip saving: " << RESULT_DIR + dense_map_file_name_);
            return;
        }

        const std::string dense_map_path = RESULT_DIR + dense_map_file_name_;
        const int save_status = pcl::io::savePCDFileBinaryCompressed(dense_map_path, dense_map);
        if (save_status != 0) {
            ROS_WARN_STREAM("failed to save dense optimized map: " << dense_map_path
                            << ", pcl_status=" << save_status);
            return;
        }

        ROS_INFO_STREAM("saved dense optimized map to " << dense_map_path
                        << " with " << dense_map.size() << " points");
    }

    void IESKFBackEndWrapper::optimizedMapPublishLoop()
    {
        using Clock = std::chrono::steady_clock;
        const auto min_publish_interval =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / optimized_map_publish_hz_));
        auto next_publish_time = Clock::now();

        while (true) {
            ros::Time stamp;
            std::uint64_t target_version = 0;
            {
                std::unique_lock<std::mutex> lock(optimized_map_state_mutex_);
                optimized_map_cv_.wait(lock, [this]() {
                    return stop_optimized_map_thread_ || optimized_map_version_ > published_map_version_;
                });
                if (stop_optimized_map_thread_) {
                    return;
                }

                const auto now = Clock::now();
                if (now < next_publish_time) {
                    optimized_map_cv_.wait_until(lock, next_publish_time, [this]() {
                        return stop_optimized_map_thread_;
                    });
                    if (stop_optimized_map_thread_) {
                        return;
                    }
                }

                target_version = optimized_map_version_;
                stamp = latest_optimized_map_stamp_;
            }

            IESKFSLAM::PCLPointCloud optimized_map;
            {
                std::lock_guard<std::mutex> lock(backend_mutex_);
                optimized_map = backend_coordinator.buildOptimizedMap();
            }

            sensor_msgs::PointCloud2 map_msg;
            pcl::toROSMsg(optimized_map, map_msg);
            map_msg.header.stamp = stamp;
            map_msg.header.frame_id = "map";
            optimized_map_pub.publish(map_msg);

            {
                std::lock_guard<std::mutex> lock(optimized_map_state_mutex_);
                published_map_version_ = target_version;
            }
            next_publish_time = Clock::now() + min_publish_interval;
        }
    }
} // namespace ROSNoetic
