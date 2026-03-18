#include"ieskf_slam/modules/map/rect_map_manager.h"
#include <cstddef>
#include <memory>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include "ieskf_slam/math/math.h"
namespace IESKFSLAM{
    RectMapManager::RectMapManager(const std::string&config_path,const std::string&prefix):ModuleBase(config_path, prefix, "RectMapManager"){
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KdTree>();
        readParam<float>("map_side_length_2", map_side_length_2, 500);
        readParam<float>("map_resolution", map_resolution, 0.5);
    }
    RectMapManager::~RectMapManager(){}
    void RectMapManager::addScan(const PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        if(local_map_ptr->empty()){
        *local_map_ptr += scan;
        }
        else{
            for(auto&& point:scan){
                std::vector<int> point_ind;
                std::vector<float> distance;
                kdtree_ptr->nearestKSearch(point, 5, point_ind, distance);
                if(distance[0] > map_resolution){
                    local_map_ptr->push_back(point);
                }
            }

            size_t left {0};
            size_t right {local_map_ptr->size()-1};
            while(right > left){
                while (right > left && (   std::abs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2
                                        || std::abs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2
                                        || std::abs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)) {--right;}
                while (right > left && (   std::abs(local_map_ptr->points[left].x - pos_t.x()) < map_side_length_2
                                        && std::abs(local_map_ptr->points[left].y - pos_t.y()) < map_side_length_2
                                        && std::abs(local_map_ptr->points[left].z - pos_t.z()) < map_side_length_2)) {++left;}
                std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);      
            }
            local_map_ptr->resize(right+1);
    }
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
