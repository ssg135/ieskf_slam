#include <glog/logging.h>
#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    ros::init(argc, argv, "ieskf_slam_node");
    ros::NodeHandle nh;

    auto front_end_wrapper_ptr = std::make_shared<ROSNoetic::IESKFFrontEndWrapper>(nh);
    auto back_end_wrapper_ptr = std::make_shared<ROSNoetic::IESKFBackEndWrapper>(nh);

    ros::Rate rate(500);
    while (ros::ok()) {
        ros::spinOnce();
        front_end_wrapper_ptr->spinOnce();
        rate.sleep();
    }

    back_end_wrapper_ptr.reset();
    front_end_wrapper_ptr.reset();
    google::ShutdownGoogleLogging();
    return 0;
}
