#include"ieskf_slam/modules/ieskf/ieskf.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include "ieskf_slam/math/SO3.hpp"

namespace IESKFSLAM{
    IESKF::IESKF(const std::string&config_path,const std::string&prefix): ModuleBase(config_path, prefix, "IESKF"){
        P.setIdentity();
        P(9,9) = P(10,10) = P(11,11) = 0.0001;
        P(12,12) = P(12,12) = P(13,13) = 0.001;
        P(14,14) = P(15,15) = P(16,16) = 0.00001;
        double cov_gyroscope, cov_acceleration, cov_bias_gyroscope, cov_bias_acceleration;
        readParam("cov_gyroscope", cov_gyroscope, 0.1);
        readParam("cov_acceleration", cov_acceleration, 0.1);
        readParam("cov_gyroscope", cov_bias_gyroscope, 0.1);
        readParam("cov_gyroscope", cov_bias_acceleration, 0.1);
        Q.block<3,3>(0,0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3,3>(3,3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3,3>(6,6).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3,3>(9,9).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        
    }
    IESKF::~IESKF(){}
    void IESKF::predict(IMU &imu, double dt){
        imu.acceleration -= X.ba;
        imu.gyroscope -= X.bg;
        auto rotation = X.rotation.toRotationMatrix();
        X.rotation = Eigen::Quaterniond(rotation * so3Exp(imu.gyroscope*dt));
        X.rotation.normalize();
        X.position += X.velocity*dt;
        X.velocity += rotation*(imu.acceleration+X.gravity)*dt;

        Eigen::Matrix<double,18,18> Fx;
        Eigen::Matrix<double,18,12> Fw;
        Fx.setIdentity();
        Fw.setZero();

        Fx.block<3,3>(0,0) = so3Exp(-(imu.gyroscope - X.bg)*dt);
        Fx.block<3,3>(0,9) = -A_T((imu.gyroscope - X.bg)*dt)*dt;
        Fx.block<3,3>(6,0) = -rotation * skewSymmetric(imu.acceleration-X.ba)*dt;
        Fx.block<3,3>(3,6) = Eigen::Matrix3d::Identity()*dt;
        Fx.block<3,3>(6,12) = -rotation*dt;
        Fx.block<3,3>(6,15) = Eigen::Matrix3d::Identity()*dt;

        Fw.block<3,3>(0,0) = -A_T((imu.gyroscope - X.bg)*dt)*dt;
        Fw.block<3,3>(6,3) = -rotation*dt;
        Fw.block<3,3>(9,6) = Eigen::Matrix3d::Identity()*dt;
        Fw.block<3,3>(12,9) = Eigen::Matrix3d::Identity()*dt;

        P = Fx*P*Fx.transpose()+Fw*Q*Fw.transpose(); 
    }
    bool IESKF::update(){
        return true;
    }
    const IESKF::State18& IESKF::getX(){
        return X;
    }
    void IESKF::setX(const State18 &x_in){
        X = x_in;
    }
}