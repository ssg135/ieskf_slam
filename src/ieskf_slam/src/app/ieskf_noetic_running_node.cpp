
#include <glog/logging.h>
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    ros::init(argc,argv,"front_end_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNoetic::IESKFFrontEndWrapper>front_end_wrapper_ptr;
    front_end_wrapper_ptr = std::make_shared<ROSNoetic::IESKFFrontEndWrapper>(nh);
    front_end_wrapper_ptr->run();
    google::ShutdownGoogleLogging();
    return 0;
}
