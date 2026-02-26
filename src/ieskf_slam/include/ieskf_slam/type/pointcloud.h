#pragma once
#include "ieskf_slam/type/timestamp.h"
#include "ieskf_slam/type/point.h"
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>

namespace IESKFSLAM{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = pcl::PointCloud<Point>::Ptr;
    using PCLPointCloudConstPtr = pcl::PointCloud<Point>::ConstPtr;
    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        PCLPointCloudPtr cloud_ptr;
        Timestamp time_stamp;
        PointCloud(){
            cloud_ptr = boost::make_shared<PCLPointCloud>();
        }
    };
}