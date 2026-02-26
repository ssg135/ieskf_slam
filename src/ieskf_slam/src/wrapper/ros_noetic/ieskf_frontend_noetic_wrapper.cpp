#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"
#include <iostream>

namespace ROSNoetic{
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle& nh){
        std::string lidar_topic, imu_topic, config_file_name;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        std::cout<<"lidar_topic: "<<lidar_topic<<std::endl;
        std::cout<<"imu_topic: "<<imu_topic<<std::endl;

        front_end_ptr = std::make_shared<IESKFSLAM::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

        cloud_subscriber = nh.subscribe(lidar_topic, 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        current_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);

        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if(lidar_type == AVIA){
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        }
        else{
            std::cout<<"unsupport lidar type"<<std::endl;
            exit(100);
        }

        run();
    }
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper(){}

    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        IESKFSLAM::PointCloud cloud;
        lidar_process_ptr->process(*msg, cloud);
        front_end_ptr->addPointCloud(cloud);
    }
    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuConstPtr& msg){
        IESKFSLAM::IMU imu;
        imu.acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        imu.gyroscope << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        imu.time_stamp.fromNSec(msg->header.stamp.toNSec());
        front_end_ptr->addImu(imu);
    }
    void IESKFFrontEndWrapper::run(){
        ros::Rate rate(500);
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
            if(front_end_ptr->track()){
                publishMsg();
            }
        }
    }
    void IESKFFrontEndWrapper::publishMsg(){
        
    }

}