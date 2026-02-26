#pragma once
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
#include <deque>

namespace IESKFSLAM{
    struct MeasureGroup{
        double lidar_begin_time;
        double lidar_end_time;
        std::deque<IMU> imus;
        PointCloud point_cloud;
    };
}