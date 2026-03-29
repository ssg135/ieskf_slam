#include "ieskf_slam/modules/map/map_manager_base.h"
#include "ieskf_slam/common/logging.h"
#include "ieskf_slam/modules/map/ikdtree_map_manager.h"
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include <algorithm>
#include <cctype>
#include <yaml-cpp/yaml.h>

namespace IESKFSLAM {

namespace {

std::string normalizeType(std::string type) {
    std::transform(type.begin(), type.end(), type.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return type;
}

std::string loadMapType(const std::string& config_path, const std::string& prefix) {
    if (config_path.empty()) {
        return "rect_kdtree";
    }
    try {
        YAML::Node config_node = YAML::LoadFile(config_path);
        if (!prefix.empty() && config_node[prefix]) {
            config_node = config_node[prefix];
        }
        if (config_node["type"]) {
            return config_node["type"].as<std::string>();
        }
    } catch (const YAML::Exception& e) {
        SLAM_LOG_WARN << "Failed to load map config from " << config_path << ": " << e.msg;
    }
    return "rect_kdtree";
}

}  // namespace

MapManagerBase::Ptr CreateMapManager(const std::string& config_path, const std::string& prefix) {
    const std::string type = normalizeType(loadMapType(config_path, prefix));
    if (type == "ikdtree" || type == "ikd_tree") {
        SLAM_LOG_INFO << "Using IKDTree map manager";
        return std::make_shared<IKDTreeMapManager>(config_path, prefix);
    }
    if (type != "rect_kdtree" && type != "rect" && type != "pcl_kdtree") {
        SLAM_LOG_WARN << "Unknown map manager type '" << type
                      << "', fallback to rect_kdtree";
    }
    SLAM_LOG_INFO << "Using Rect/PCL KdTree map manager";
    return std::make_shared<RectMapManager>(config_path, prefix);
}

}  // namespace IESKFSLAM
