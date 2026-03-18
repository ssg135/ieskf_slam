#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "geometry_msgs/PoseStamped.h"
#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/math/math.h"
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>

namespace ROSNoetic{
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle& nh){
        std::string lidar_topic, imu_topic, config_file_name;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        ROS_INFO_STREAM("lidar_topic: " << lidar_topic);
        ROS_INFO_STREAM("imu_topic: " << imu_topic);
        ROS_INFO_STREAM("config_file_name: " << config_file_name);

        front_end_ptr = std::make_shared<IESKFSLAM::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

        cloud_subscriber = nh.subscribe(lidar_topic, 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        current_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
        current_local_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
        path_publisher = nh.advertise<nav_msgs::Path>("path", 100);
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if(lidar_type == AVIA){
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        }
        else{
            ROS_ERROR_STREAM("unsupported lidar type: " << lidar_type);
            exit(100);
        }

        run();
    }
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper(){}

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
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped pst;
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
        msg.header.frame_id = "map";
        current_pointcloud_publisher.publish(msg);
        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud,msg);
        msg.header.frame_id = "map";
        current_local_map_publisher.publish(msg);
        ROS_DEBUG_STREAM_THROTTLE(1.0, "publish path_size=" << path.poses.size()
                                  << ", cloud_size=" << cloud.size()
                                  << ", position=[" << X.position.transpose() << "]");
    }

}
