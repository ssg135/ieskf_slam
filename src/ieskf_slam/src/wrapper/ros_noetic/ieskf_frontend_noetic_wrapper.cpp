#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "geometry_msgs/PoseStamped.h"
#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/math/math.h"
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <iomanip>
#include <sstream>

namespace ROSNoetic{
    namespace {
        constexpr double kWarnCloudWithPosePublishGapSec = 5.0;
    }

    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle& nh){
        std::string lidar_topic, imu_topic, config_file_name;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        ROS_DEBUG_STREAM("lidar_topic: " << lidar_topic);
        ROS_DEBUG_STREAM("imu_topic: " << imu_topic);
        ROS_DEBUG_STREAM("config_file_name: " << config_file_name);

        front_end_ptr = std::make_shared<IESKFSLAM::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

        cloud_subscriber = nh.subscribe(lidar_topic, 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        current_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
        current_local_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
        cloud_with_pose_publisher = nh.advertise<ieskf_slam::CloudWithPose>("cloud_with_pose",100);
        path_publisher = nh.advertise<nav_msgs::Path>("path", 100);
        nh.param<std::string>("front_end/anomaly_log_file_name", anomaly_log_file_name_,
                              "frontend_publish_anomalies.txt");
        anomaly_log_file_.open(RESULT_DIR + anomaly_log_file_name_, std::ios::out | std::ios::trunc);
        if (!anomaly_log_file_.is_open()) {
            ROS_WARN_STREAM("failed to open frontend anomaly log file: " << RESULT_DIR + anomaly_log_file_name_);
        } else {
            anomaly_log_file_ << std::setprecision(15);
        }
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if(lidar_type == AVIA){
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        }
        else if(lidar_type == VELO){
            lidar_process_ptr = std::make_shared<VelodyneProcess>();
        }
        else if(lidar_type == GENERIC_POINTCLOUD2){
            lidar_process_ptr = std::make_shared<GenericPointCloud2Process>();
        }
        else{
            ROS_ERROR_STREAM("unsupported lidar type: " << lidar_type);
            exit(100);
        }

        run();
    }
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper(){
        if (anomaly_log_file_.is_open()) {
            anomaly_log_file_.flush();
            anomaly_log_file_.close();
        }
    }

    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        IESKFSLAM::PointCloud cloud = lidar_process_ptr->process(*msg);
        ROS_DEBUG_STREAM_THROTTLE(1.0, "lidar callback stamp=" << msg->header.stamp.toSec()
                                  << ", raw_width=" << msg->width
                                  << ", raw_height=" << msg->height
                                  << ", parsed_points=" << cloud.cloud_ptr->size());
        front_end_ptr->addPointCloud(cloud);
    }
    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuConstPtr& msg){
        IESKFSLAM::IMU imu;
        imu.acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        imu.gyroscope << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        imu.time_stamp.fromNSec(msg->header.stamp.toNSec());
        ROS_DEBUG_STREAM_THROTTLE(1.0, "imu callback stamp=" << msg->header.stamp.toSec()
                                  << ", acc=[" << imu.acceleration.transpose() << "]"
                                  << ", gyro=[" << imu.gyroscope.transpose() << "]");
        front_end_ptr->addImu(imu);
    }
    void IESKFFrontEndWrapper::run(){
        ros::Rate rate(500);
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
            if(front_end_ptr->track()){
                publishMsg();
            } else {
                ROS_DEBUG_THROTTLE(1.0, "front_end track() did not produce a publishable state");
            }
        }
    }
    void IESKFFrontEndWrapper::publishMsg(){
        static nav_msgs::Path path;
        auto X = front_end_ptr->readState();
        const ros::Time frame_stamp(front_end_ptr->readCurrentFrameStamp().sec());
        path.header.frame_id = "map";
        path.header.stamp = frame_stamp;
        geometry_msgs::PoseStamped pst;
        pst.header = path.header;
        pst.pose.position.x = X.position.x();
        pst.pose.position.y = X.position.y();
        pst.pose.position.z = X.position.z();
        path.poses.push_back(pst);
        path_publisher.publish(path);
        IESKFSLAM::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
        pcl::transformPointCloud(cloud,cloud,IESKFSLAM::compositeTransform(X.rotation,X.position).cast<float>());
        // auto cloud =front_end_ptr->readCurrentPointCloud();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud,msg);
        msg.header.stamp = frame_stamp;
        msg.header.frame_id = "map";
        current_pointcloud_publisher.publish(msg);
        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud,msg);
        msg.header.stamp = frame_stamp;
        msg.header.frame_id = "map";
        current_local_map_publisher.publish(msg);
        ieskf_slam::CloudWithPose cloud_with_pose_msg;
        cloud = front_end_ptr->readFullPointCloud();
        pcl::toROSMsg(cloud, cloud_with_pose_msg.point_cloud);
        cloud_with_pose_msg.pose.position.x = X.position.x();
        cloud_with_pose_msg.pose.position.y = X.position.y();
        cloud_with_pose_msg.pose.position.z = X.position.z();
        cloud_with_pose_msg.pose.orientation.w = X.rotation.w();
        cloud_with_pose_msg.pose.orientation.x = X.rotation.x();
        cloud_with_pose_msg.pose.orientation.y = X.rotation.y();
        cloud_with_pose_msg.pose.orientation.z = X.rotation.z();
        cloud_with_pose_msg.point_cloud.header.stamp = frame_stamp;
        cloud_with_pose_msg.point_cloud.header.frame_id = "lidar";
        if (!last_cloud_with_pose_publish_stamp_.isZero()) {
            const double publish_gap_sec =
                (cloud_with_pose_msg.point_cloud.header.stamp - last_cloud_with_pose_publish_stamp_).toSec();
            if (publish_gap_sec > kWarnCloudWithPosePublishGapSec) {
                ROS_WARN_STREAM("frontend cloud_with_pose publish gap: dt=" << publish_gap_sec
                                << " s, stamp=" << cloud_with_pose_msg.point_cloud.header.stamp.toSec()
                                << ", path_size=" << path.poses.size()
                                << ", cloud_size=" << cloud.size());
                std::ostringstream oss;
                oss << "frontend cloud_with_pose publish gap"
                    << " stamp=" << cloud_with_pose_msg.point_cloud.header.stamp.toSec()
                    << " dt=" << publish_gap_sec
                    << " path_size=" << path.poses.size()
                    << " cloud_size=" << cloud.size();
                writeAnomalyLogLine(oss.str());
            }
        }
        last_cloud_with_pose_publish_stamp_ = cloud_with_pose_msg.point_cloud.header.stamp;
        cloud_with_pose_publisher.publish(cloud_with_pose_msg);
        ROS_DEBUG_STREAM_THROTTLE(1.0, "publish path_size=" << path.poses.size()
                                  << ", cloud_size=" << cloud.size()
                                  << ", position=[" << X.position.transpose() << "]");
    }

    void IESKFFrontEndWrapper::writeAnomalyLogLine(const std::string& line){
        if (!anomaly_log_file_.is_open()) {
            return;
        }
        anomaly_log_file_ << line << std::endl;
        anomaly_log_file_.flush();
    }

}
