#pragma once
#include <Eigen/Dense>
#include "ieskf_slam/type/timestamp.h"

namespace IESKFSLAM{
    class IMU{
        public:
            Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();
            Timestamp time_stamp;
            void clear(){
                acceleration = Eigen::Vector3d::Zero();
                gyroscope = Eigen::Vector3d::Zero();
                time_stamp.clear();
            }
            IMU operator+(const IMU& other){
                IMU result;
                result.acceleration = this->acceleration + other.acceleration;
                result.gyroscope = this->gyroscope + other.gyroscope;
                return result;
            }
            IMU operator*(double k){
                IMU result;
                result.acceleration = this->acceleration * k;
                result.gyroscope = this->gyroscope * k;
                return result;
            }
            IMU operator/(double k){
                IMU result;
                result.acceleration = this->acceleration / k;
                result.gyroscope = this->gyroscope / k;
                return result;
            }
            friend std::ostream& operator<<(std::ostream& ostream, const IMU& imu){
                ostream<<"imu_time: "<<imu.time_stamp.sec()<<" s"<<" |acc: "<<imu.acceleration.transpose()<<" |gyro: "<<imu.gyroscope.transpose()<<std::endl;
                return ostream;
            }

    };
}