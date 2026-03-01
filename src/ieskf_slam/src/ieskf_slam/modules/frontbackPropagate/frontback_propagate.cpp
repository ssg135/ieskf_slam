#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"
#include <algorithm>

namespace IESKFSLAM {
    FrontbackPropagate::FrontbackPropagate(){}
    FrontbackPropagate::~FrontbackPropagate(){}
    void FrontbackPropagate::propagate(MeasureGroup &mg, IESKF::Ptr ieskf_ptr){
        std::sort(mg.point_cloud.cloud_ptr->points.begin(), mg.point_cloud.cloud_ptr->points.end(),
        [](Point x, Point y){return x.offset_time < y.offset_time;});

        mg.imus.push_front(last_imu);
        IMU in;
        double dt{0};
        for(auto it_imu = mg.imus.begin(); it_imu < mg.imus.end()-1; it_imu++ ){
            auto&& head = *(it_imu);
            auto&& tail = *(it_imu + 1);
            dt = tail.time_stamp.sec() - head.time_stamp.sec();
            auto avr_gyro =0.5*(head.gyroscope + tail.gyroscope);
            auto avr_acc =0.5*(head.acceleration + tail.acceleration)*imu_scale;
            in.gyroscope = avr_gyro;
            in.acceleration = avr_acc;
            ieskf_ptr->predict(in, dt);
        }

        dt = mg.lidar_end_time - mg.imus.back().time_stamp.sec();
        ieskf_ptr->predict(in, dt);
        last_imu = mg.imus.back();
        last_imu.time_stamp.fromSec(mg.lidar_end_time);
    }
}