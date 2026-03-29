#include "ieskf_slam/modules/map/ikdtree_map_manager.h"
#include "ieskf_slam/math/math.h"
#include <algorithm>
#include <pcl/common/transforms.h>

namespace IESKFSLAM {

IKDTreeMapManager::IKDTreeMapManager(const std::string& config_path, const std::string& prefix)
    : MapManagerBase(config_path, prefix, "IKDTreeMapManager") {
    local_map_ptr_ = pcl::make_shared<PCLPointCloud>();
    readParam<float>("map_side_length_2", map_side_length_2_, 500.0f);
    readParam<float>("map_resolution", map_resolution_, 0.5f);
    readParam<float>("ikdtree_delete_criterion", ikdtree_delete_criterion_, 0.5f);
    readParam<float>("ikdtree_balance_criterion", ikdtree_balance_criterion_, 0.7f);
    readParam<float>("ikdtree_downsample_size", ikdtree_downsample_size_, map_resolution_);
    ikdtree_ptr_ = std::make_unique<KD_TREE>(ikdtree_delete_criterion_, ikdtree_balance_criterion_,
                                             ikdtree_downsample_size_);
    print_table();
}

IKDTreeMapManager::~IKDTreeMapManager() = default;

::PointType IKDTreeMapManager::toIKDPoint(const Point& point) {
    ::PointType ikd_point;
    ikd_point.x = point.x;
    ikd_point.y = point.y;
    ikd_point.z = point.z;
    ikd_point.intensity = point.intensity;
    ikd_point.normal_x = 0.0f;
    ikd_point.normal_y = 0.0f;
    ikd_point.normal_z = 0.0f;
    ikd_point.curvature = 0.0f;
    return ikd_point;
}

Point IKDTreeMapManager::fromIKDPoint(const ::PointType& point) {
    Point slam_point;
    slam_point.x = point.x;
    slam_point.y = point.y;
    slam_point.z = point.z;
    slam_point.intensity = point.intensity;
    slam_point.offset_time = 0;
    slam_point.ring = 0;
    return slam_point;
}

void IKDTreeMapManager::reset() {
    local_map_ptr_->clear();
    ikdtree_ptr_ = std::make_unique<KD_TREE>(ikdtree_delete_criterion_, ikdtree_balance_criterion_,
                                             ikdtree_downsample_size_);
}

std::vector<BoxPointType> IKDTreeMapManager::buildDeleteBoxes(const Eigen::Vector3d& pos_t) const {
    std::vector<BoxPointType> boxes;
    if (!ikdtree_ptr_ || ikdtree_ptr_->size() == 0) {
        return boxes;
    }

    const BoxPointType range = ikdtree_ptr_->tree_range();
    const float min_x = static_cast<float>(pos_t.x() - map_side_length_2_);
    const float max_x = static_cast<float>(pos_t.x() + map_side_length_2_);
    const float min_y = static_cast<float>(pos_t.y() - map_side_length_2_);
    const float max_y = static_cast<float>(pos_t.y() + map_side_length_2_);
    const float min_z = static_cast<float>(pos_t.z() - map_side_length_2_);
    const float max_z = static_cast<float>(pos_t.z() + map_side_length_2_);
    const float padding = 1e-3f;

    auto push_box = [&boxes](float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
        BoxPointType box;
        box.vertex_min[0] = xmin;
        box.vertex_min[1] = ymin;
        box.vertex_min[2] = zmin;
        box.vertex_max[0] = xmax;
        box.vertex_max[1] = ymax;
        box.vertex_max[2] = zmax;
        boxes.push_back(box);
    };

    if (range.vertex_min[0] < min_x) {
        push_box(range.vertex_min[0] - padding, min_x, range.vertex_min[1] - padding,
                 range.vertex_max[1] + padding, range.vertex_min[2] - padding,
                 range.vertex_max[2] + padding);
    }
    if (range.vertex_max[0] > max_x) {
        push_box(max_x, range.vertex_max[0] + padding, range.vertex_min[1] - padding,
                 range.vertex_max[1] + padding, range.vertex_min[2] - padding,
                 range.vertex_max[2] + padding);
    }
    if (range.vertex_min[1] < min_y) {
        push_box(min_x, max_x, range.vertex_min[1] - padding, min_y, range.vertex_min[2] - padding,
                 range.vertex_max[2] + padding);
    }
    if (range.vertex_max[1] > max_y) {
        push_box(min_x, max_x, max_y, range.vertex_max[1] + padding, range.vertex_min[2] - padding,
                 range.vertex_max[2] + padding);
    }
    if (range.vertex_min[2] < min_z) {
        push_box(min_x, max_x, min_y, max_y, range.vertex_min[2] - padding, min_z);
    }
    if (range.vertex_max[2] > max_z) {
        push_box(min_x, max_x, min_y, max_y, max_z, range.vertex_max[2] + padding);
    }
    return boxes;
}

void IKDTreeMapManager::pruneLocalMap(const Eigen::Vector3d& pos_t) {
    auto& points = local_map_ptr_->points;
    points.erase(std::remove_if(points.begin(), points.end(), [&](const Point& point) {
        return std::abs(point.x - pos_t.x()) > map_side_length_2_ ||
               std::abs(point.y - pos_t.y()) > map_side_length_2_ ||
               std::abs(point.z - pos_t.z()) > map_side_length_2_;
    }), points.end());
    local_map_ptr_->width = static_cast<std::uint32_t>(points.size());
    local_map_ptr_->height = 1;
    local_map_ptr_->is_dense = true;
}

bool IKDTreeMapManager::nearestKSearch(const Point& query_point, int k, std::vector<Point>& nearest_points,
                                       std::vector<float>& squared_distances) const {
    nearest_points.clear();
    squared_distances.clear();
    if (!ikdtree_ptr_ || ikdtree_ptr_->size() == 0 || k <= 0) {
        return false;
    }
    PointVector nearest_ikd_points;
    std::vector<float> distances;
    ikdtree_ptr_->Nearest_Search(toIKDPoint(query_point), k, nearest_ikd_points, distances);
    nearest_points.reserve(nearest_ikd_points.size());
    for (const auto& point : nearest_ikd_points) {
        nearest_points.push_back(fromIKDPoint(point));
    }
    squared_distances = std::move(distances);
    return !nearest_points.empty();
}

void IKDTreeMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q,
                                const Eigen::Vector3d& pos_t) {
    PCLPointCloud scan_world;
    pcl::transformPointCloud(*curr_scan, scan_world, compositeTransform(att_q, pos_t).cast<float>());
    if (scan_world.empty()) {
        return;
    }

    const auto delete_boxes = buildDeleteBoxes(pos_t);
    if (!delete_boxes.empty()) {
        auto mutable_boxes = delete_boxes;
        ikdtree_ptr_->Delete_Point_Boxes(mutable_boxes);
        pruneLocalMap(pos_t);
    }

    PointVector points_to_add;
    points_to_add.reserve(scan_world.size());
    const float distance_threshold_sq = map_resolution_ * map_resolution_;
    for (const auto& point : scan_world.points) {
        std::vector<Point> nearest_points;
        std::vector<float> distances;
        if (!nearestKSearch(point, 1, nearest_points, distances) || distances.empty() ||
            distances.front() > distance_threshold_sq) {
            local_map_ptr_->push_back(point);
            points_to_add.push_back(toIKDPoint(point));
        }
    }

    if (!points_to_add.empty()) {
        if (ikdtree_ptr_->size() == 0) {
            ikdtree_ptr_->Build(points_to_add);
        } else {
            ikdtree_ptr_->Add_Points(points_to_add, false);
        }
    }

    local_map_ptr_->width = static_cast<std::uint32_t>(local_map_ptr_->size());
    local_map_ptr_->height = 1;
    local_map_ptr_->is_dense = true;
}

PCLPointCloudConstPtr IKDTreeMapManager::getLocalMap() const {
    return local_map_ptr_;
}

}  // namespace IESKFSLAM
