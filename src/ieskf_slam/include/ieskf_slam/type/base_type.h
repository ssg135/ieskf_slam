#pragma once 
#include "ieskf_slam/type/point.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace IESKFSLAM{
    using VoxelFilter = pcl::VoxelGrid<Point>;
    using KdTree = pcl::KdTreeFLANN<Point>;
    using KdTreePtr = KdTree::Ptr;
    const double GRAVITY = 9.81;
}