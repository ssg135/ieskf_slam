#pragma once
#include "ieskf_slam/type/timestamp.h"
#include "ieskf_slam/type/point.h"
#include <boost/make_shared.hpp>

namespace IESKFSLAM{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = pcl::PointCloud<Point>::Ptr;
    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        PCLPointCloudPtr cloud_ptr;
        Timestamp time_stamp;
        PointCloud(){
            cloud_ptr = boost::make_shared<PCLPointCloud>();
        }
    };
}