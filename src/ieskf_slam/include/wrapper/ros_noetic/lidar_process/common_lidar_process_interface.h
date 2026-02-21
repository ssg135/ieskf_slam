#pragma once
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/pointcloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic
{
    class CommonLidarProcessInterface
    {
    public:
        // 根据不同的lidar 转换成统一的cloud
        virtual bool process(const sensor_msgs::PointCloud2 &msg, IESKFSLAM::PointCloud &cloud) = 0;
    };
} // namespace ROSNoetic

