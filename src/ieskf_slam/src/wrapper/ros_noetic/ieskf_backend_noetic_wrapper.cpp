#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"

#include "geometry_msgs/PoseStamped.h"
#include "ieskf_slam/globaldefine.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>

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
        keyframe_map_pub = nh.advertise<sensor_msgs::PointCloud2>("backend/keyframe_map", 1, true);
        keyframe_path_pub = nh.advertise<nav_msgs::Path>("backend/keyframe_path", 1, true);
        keyframe_positions_pub = nh.advertise<sensor_msgs::PointCloud2>("backend/keyframe_positions", 1, true);
        nh.param<bool>("back_end/enable_save_dense_map", enable_save_dense_map_, true);
        nh.param<int>("back_end/loop_skip_after_success_keyframes", loop_skip_after_success_keyframes_, 30);
        nh.param<std::string>("back_end/dense_map_file_name", dense_map_file_name_, "optimized_map_dense.pcd");
        keyframe_worker_thread_ = std::thread(&IESKFBackEndWrapper::processPendingKeyframes, this);
        if (backend_coordinator.isLoopClosureEnabled()) {
            loop_worker_thread_ = std::thread(&IESKFBackEndWrapper::processPendingLoops, this);
        }
    }

    IESKFBackEndWrapper::~IESKFBackEndWrapper()
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_requested_ = true;
        }
        worker_cv_.notify_all();
        loop_cv_.notify_all();

        if (keyframe_worker_thread_.joinable()) {
            keyframe_worker_thread_.join();
        }
        if (loop_worker_thread_.joinable()) {
            loop_worker_thread_.join();
        }

        saveDenseMapToFile();
    }

    void IESKFBackEndWrapper::CloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPose::ConstPtr& msg)
    {
        std::size_t pending_cloud_count = 0;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            pending_cloud_msgs_.push_back(msg);
            pending_cloud_count = pending_cloud_msgs_.size();
        }
        worker_cv_.notify_one();
        if (pending_cloud_count > 10) {
            ROS_WARN_STREAM_THROTTLE(1.0, "backend pending cloud_with_pose queue=" << pending_cloud_count);
        }
    }

    void IESKFBackEndWrapper::processPendingKeyframes()
    {
        while (true) {
            ieskf_slam::CloudWithPose::ConstPtr msg;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                worker_cv_.wait(lock, [this]() {
                    return stop_requested_ || !pending_cloud_msgs_.empty();
                });

                if (stop_requested_ && pending_cloud_msgs_.empty()) {
                    return;
                }

                msg = pending_cloud_msgs_.front();
                pending_cloud_msgs_.pop_front();
            }

            IESKFSLAM::Pose pose;
            pose.position.x() = msg->pose.position.x;
            pose.position.y() = msg->pose.position.y;
            pose.position.z() = msg->pose.position.z;
            pose.rotation.w() = msg->pose.orientation.w;
            pose.rotation.x() = msg->pose.orientation.x;
            pose.rotation.y() = msg->pose.orientation.y;
            pose.rotation.z() = msg->pose.orientation.z;
            pose.time_stamp.fromNSec(msg->point_cloud.header.stamp.toNSec());

            if (!backend_coordinator.shouldCreateKeyframe(pose)) {
                continue;
            }

            IESKFSLAM::PCLPointCloud cloud;
            pcl::fromROSMsg(msg->point_cloud, cloud);
            if (cloud.empty()) {
                continue;
            }

            const IESKFSLAM::BackendProcessResult result =
                backend_coordinator.processKeyframe(cloud, pose, msg->point_cloud.header.stamp.toSec());
            if (!result.inserted_keyframe) {
                continue;
            }

            const std::vector<IESKFSLAM::Pose> keyframe_poses = backend_coordinator.readOptimizedPoses();
            publishKeyframePath(keyframe_poses, msg->point_cloud.header.stamp);
            publishKeyframePositions(keyframe_poses, msg->point_cloud.header.stamp);

            if (keyframe_map_pub.getNumSubscribers() > 0) {
                publishKeyframeMap(msg->point_cloud.header.stamp);
            }

            if (backend_coordinator.isLoopClosureEnabled()) {
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    pending_loop_task_ = {result.keyframe_id, msg->point_cloud.header.stamp};
                    has_pending_loop_task_ = true;
                }
                loop_cv_.notify_one();
            }
        }
    }

    void IESKFBackEndWrapper::processPendingLoops()
    {
        int last_processed_keyframe_id = -1;
        int last_successful_loop_keyframe_id = -1000000;
        while (true) {
            LoopTask task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                loop_cv_.wait(lock, [this]() {
                    return stop_requested_ || has_pending_loop_task_;
                });

                if (stop_requested_ && !has_pending_loop_task_) {
                    return;
                }

                task = pending_loop_task_;
                has_pending_loop_task_ = false;
            }

            if (task.keyframe_id <= last_processed_keyframe_id) {
                continue;
            }
            last_processed_keyframe_id = task.keyframe_id;

            if (task.keyframe_id - last_successful_loop_keyframe_id <
                std::max(loop_skip_after_success_keyframes_, 0)) {
                continue;
            }

            const IESKFSLAM::BackendProcessResult result =
                backend_coordinator.processLoopClosure(task.keyframe_id);
            if (!result.accepted_loop) {
                continue;
            }
            last_successful_loop_keyframe_id = task.keyframe_id;

            if (!result.optimized) {
                continue;
            }

            const std::vector<IESKFSLAM::Pose> keyframe_poses = backend_coordinator.readOptimizedPoses();
            publishKeyframePath(keyframe_poses, task.stamp);
            publishKeyframePositions(keyframe_poses, task.stamp);

            if (keyframe_map_pub.getNumSubscribers() > 0) {
                publishKeyframeMap(task.stamp);
            }
        }
    }

    void IESKFBackEndWrapper::publishKeyframePath(const std::vector<IESKFSLAM::Pose>& keyframe_poses,
                                                  const ros::Time& stamp)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";
        path_msg.poses.reserve(keyframe_poses.size());
        for (const auto& pose : keyframe_poses) {
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
        keyframe_path_pub.publish(path_msg);
    }

    void IESKFBackEndWrapper::publishKeyframePositions(const std::vector<IESKFSLAM::Pose>& keyframe_poses,
                                                       const ros::Time& stamp)
    {
        IESKFSLAM::PCLPointCloud keyframe_positions;
        keyframe_positions.reserve(keyframe_poses.size());
        for (const auto& pose : keyframe_poses) {
            IESKFSLAM::Point point;
            point.x = static_cast<float>(pose.position.x());
            point.y = static_cast<float>(pose.position.y());
            point.z = static_cast<float>(pose.position.z());
            point.intensity = 0.0f;
            point.offset_time = 0;
            point.ring = 0;
            keyframe_positions.push_back(point);
        }

        sensor_msgs::PointCloud2 positions_msg;
        pcl::toROSMsg(keyframe_positions, positions_msg);
        positions_msg.header.stamp = stamp;
        positions_msg.header.frame_id = "map";
        keyframe_positions_pub.publish(positions_msg);
    }

    void IESKFBackEndWrapper::publishKeyframeMap(const ros::Time& stamp)
    {
        const IESKFSLAM::PCLPointCloud keyframe_map = backend_coordinator.buildOptimizedMap();
        if (keyframe_map.empty()) {
            return;
        }

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(keyframe_map, map_msg);
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = "map";
        keyframe_map_pub.publish(map_msg);
    }

    void IESKFBackEndWrapper::saveDenseMapToFile()
    {
        if (!enable_save_dense_map_) {
            return;
        }

        const IESKFSLAM::PCLPointCloud dense_map = backend_coordinator.buildDenseOptimizedMap();
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
} // namespace ROSNoetic
