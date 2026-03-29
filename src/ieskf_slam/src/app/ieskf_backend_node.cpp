#include <glog/logging.h>
#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    ros::init(argc, argv, "back_end_running_node");
    ros::NodeHandle nh;
    auto backend_wrapper_ptr = std::make_shared<ROSNoetic::IESKFBackEndWrapper>(nh);
    ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}
