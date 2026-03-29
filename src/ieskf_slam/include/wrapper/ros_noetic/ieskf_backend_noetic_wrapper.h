#pragma once
#include <ros/ros.h>
#include "ieskf_slam/CloudWithPose.h"
#include "ieskf_slam/modules/backend/backend_coordinator.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include <nav_msgs/Path.h>

namespace ROSNoetic
{
    class IESKFBackEndWrapper
    {
    private:
        IESKFSLAM::BackendCoordinator backend_coordinator;
        ros::Subscriber cloud_with_pose_sub;
        ros::Publisher optimized_map_pub;
        ros::Publisher optimized_path_pub;
        void CloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPose::ConstPtr& msg);
        void publishOptimizedResult(const ros::Time& stamp);
    public:
        IESKFBackEndWrapper(ros::NodeHandle& nh);
        ~IESKFBackEndWrapper();
    };
      
} // namespace ROSNoetic
