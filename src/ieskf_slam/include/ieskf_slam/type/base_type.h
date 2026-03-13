#pragma once 
#include "ieskf_slam/type/point.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace IESKFSLAM{
    using VoxelFilter = pcl::VoxelGrid<Point>;
    using KdTree = pcl::KdTreeFLANN<Point>;
    using KdTreePtr = KdTree::Ptr;
    using KdTreeConstPtr = KdTree::ConstPtr;
    const double GRAVITY = 9.81;
    template<typename _first, typename _second, typename _third>
    struct loss_type{
        _first first;
        _second second;
        _third third;
    };
}