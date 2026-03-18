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
        virtual ~CommonLidarProcessInterface() = default;

        // 根据不同的lidar 转换成统一的cloud
        virtual IESKFSLAM::PointCloud process(const sensor_msgs::PointCloud2 &msg) const = 0;
    };
} // namespace ROSNoetic
