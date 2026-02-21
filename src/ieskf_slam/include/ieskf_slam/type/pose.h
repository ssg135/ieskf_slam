#pragma once
#include "ieskf_slam/type/timestamp.h"
#include <Eigen/Dense>

namespace IESKFSLAM{
    struct Pose
    {
         Timestamp time_stamp;
         Eigen::Quaterniond rotation;
         Eigen::Vector3d position;
    };
    
}