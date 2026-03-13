#include"ieskf_slam/modules/map/rect_map_manager.h"
#include <memory>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include "ieskf_slam/math/math.h"
namespace IESKFSLAM{
    RectMapManager::RectMapManager(const std::string&config_path,const std::string&prefix):ModuleBase(config_path, prefix, "RectMapManager"){
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KdTree>();
    }
    RectMapManager::~RectMapManager(){}
    void RectMapManager::addScan(const PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        *local_map_ptr += scan;
        VoxelFilter filter;
        filter.setLeafSize(0.5, 0.5, 0.5);
        filter.setInputCloud(local_map_ptr);
        filter.filter(*local_map_ptr);
        kdtree_ptr->setInputCloud(local_map_ptr);
    }
    void RectMapManager::reset(){
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap(){
        return local_map_ptr;
    }
    KdTreeConstPtr RectMapManager::readKdtree(){
        return kdtree_ptr;
    }
}