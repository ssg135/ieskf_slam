#pragma once
#include "common_lidar_process_interface.h"
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace ROSNoetic
{
    class VelodyneProcess :public CommonLidarProcessInterface
    {
    private:
        /* data */
    public:
        VelodyneProcess(/* args */){}
        ~VelodyneProcess(){}
        IESKFSLAM::PointCloud process(const sensor_msgs::PointCloud2 &msg) const override{
            pcl::PointCloud<velodyne_ros::Point> velodyne_cloud;
            IESKFSLAM::PointCloud cloud;
            cloud.cloud_ptr->clear();
            pcl::fromROSMsg(msg,velodyne_cloud);
            if (velodyne_cloud.empty()) {
                cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
                return cloud;
            }
            double min_point_time = velodyne_cloud[0].time;
            for (const auto& point : velodyne_cloud) {
                min_point_time = std::min(min_point_time, static_cast<double>(point.time));
            }
            for (auto &&point : velodyne_cloud)
            {
                IESKFSLAM::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.intensity = point.intensity;
                p.ring = point.ring;
                p.offset_time = static_cast<std::uint32_t>((static_cast<double>(point.time) - min_point_time) * 1e9);
                cloud.cloud_ptr->push_back(p);
            }
            cloud.time_stamp.fromSec(msg.header.stamp.toSec() + min_point_time);
            return cloud;
        }
    };
}
