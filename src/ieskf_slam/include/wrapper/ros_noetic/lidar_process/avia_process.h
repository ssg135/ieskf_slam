#pragma once
#include "common_lidar_process_interface.h"
namespace avia_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t offset_time;
      std::uint8_t  line;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::uint8_t, line, line)
)
namespace ROSNoetic
{
    class AVIAProcess :public CommonLidarProcessInterface
    {
    private:
        /* data */
    public:
        AVIAProcess(/* args */){}
        ~AVIAProcess(){}
        IESKFSLAM::PointCloud process(const sensor_msgs::PointCloud2 &msg) const override{
            pcl::PointCloud<avia_ros::Point> avia_cloud;
            IESKFSLAM::PointCloud cloud;
            pcl::fromROSMsg(msg,avia_cloud);
            cloud.cloud_ptr->clear();
            for (auto &&point : avia_cloud)
            {
                IESKFSLAM::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.intensity = point.intensity;
                p.ring = point.line;
                p.offset_time = point.offset_time;
                cloud.cloud_ptr->push_back(p);
            }
            cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
            return cloud;
        }
    };

    
} // namespace ROSNoetic
