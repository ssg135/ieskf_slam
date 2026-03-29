#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/PoseStamped.h"

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
    }

    IESKFBackEndWrapper::~IESKFBackEndWrapper() = default;

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
        const auto result = backend_coordinator.processFrame(in_cloud, pose, msg->point_cloud.header.stamp.toSec());
        if (result.inserted_keyframe) {
            publishOptimizedResult(msg->point_cloud.header.stamp);
        }
    }

    void IESKFBackEndWrapper::publishOptimizedResult(const ros::Time& stamp)
    {
        const auto optimized_poses = backend_coordinator.readOptimizedPoses();
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

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(backend_coordinator.buildOptimizedMap(), map_msg);
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = "map";
        optimized_map_pub.publish(map_msg);
    }
} // namespace ROSNoetic
