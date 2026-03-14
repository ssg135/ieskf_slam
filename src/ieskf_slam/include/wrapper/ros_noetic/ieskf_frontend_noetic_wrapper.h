#pragma once
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "wrapper/ros_noetic/lidar_process/avia_process.h"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>

namespace ROSNoetic{
    enum LIDAR_TYPE{
        AVIA = 0,
    };
    class IESKFFrontEndWrapper{
        private:
            IESKFSLAM::FrontEnd::Ptr front_end_ptr;
            ros::Subscriber imu_subscriber;
            ros::Subscriber cloud_subscriber;
            ros::Publisher current_pointcloud_publisher;
            ros::Publisher current_local_map_publisher;
            ros::Publisher path_publisher;
            std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;
            

            IESKFSLAM::PCLPointCloud curr_pointcloud;
            Eigen::Quaterniond curr_q;
            Eigen::Vector3d curr_t;
            void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msg);
            void imuMsgCallBack(const sensor_msgs::ImuConstPtr& msg);
            void run();
            void publishMsg();
        public:
            IESKFFrontEndWrapper(ros::NodeHandle& nh);
            ~IESKFFrontEndWrapper();
    };
}