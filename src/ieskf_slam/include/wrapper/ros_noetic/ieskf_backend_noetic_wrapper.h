#pragma once
#include <ros/ros.h>
#include <string>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include "ieskf_slam/CloudWithPose.h"
#include "ieskf_slam/modules/backend/backend_coordinator.h"
#include <nav_msgs/Path.h>

namespace ROSNoetic
{
    class IESKFBackEndWrapper
    {
    private:
        struct LoopTask {
            int keyframe_id = -1;
            ros::Time stamp;
        };

        IESKFSLAM::BackendCoordinator backend_coordinator;
        ros::Subscriber cloud_with_pose_sub;
        ros::Publisher keyframe_map_pub;
        ros::Publisher keyframe_path_pub;
        ros::Publisher keyframe_positions_pub;
        std::deque<ieskf_slam::CloudWithPose::ConstPtr> pending_cloud_msgs_;
        std::mutex queue_mutex_;
        std::condition_variable worker_cv_;
        std::condition_variable loop_cv_;
        std::thread keyframe_worker_thread_;
        std::thread loop_worker_thread_;
        LoopTask pending_loop_task_;
        bool has_pending_loop_task_ = false;
        bool stop_requested_ = false;
        bool enable_save_dense_map_ = true;
        int loop_skip_after_success_keyframes_ = 30;
        std::string dense_map_file_name_ = "optimized_map_dense.pcd";
        void CloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPose::ConstPtr& msg);
        void processPendingKeyframes();
        void processPendingLoops();
        void publishKeyframePath(const std::vector<IESKFSLAM::Pose>& keyframe_poses, const ros::Time& stamp);
        void publishKeyframePositions(const std::vector<IESKFSLAM::Pose>& keyframe_poses, const ros::Time& stamp);
        void publishKeyframeMap(const ros::Time& stamp);
        void saveDenseMapToFile();
    public:
        IESKFBackEndWrapper(ros::NodeHandle& nh);
        ~IESKFBackEndWrapper();
    };
      
} // namespace ROSNoetic
