#include"ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/common/logging.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include "ieskf_slam/math/SO3.h"

namespace IESKFSLAM{
    IESKF::IESKF(const std::string&config_path,const std::string&prefix): ModuleBase(config_path, prefix, "IESKF"){
        P.setIdentity();
        P(9,9) = P(10,10) = P(11,11) = 0.0001;
        P(12,12) = P(13,13) = P(14,14) = 0.001;
        P(15,15) = P(16,16) = P(17,17) = 0.00001;
        double cov_gyroscope, cov_acceleration, cov_bias_gyroscope, cov_bias_acceleration;
        readParam("cov_gyroscope", cov_gyroscope, 0.1);
        readParam("cov_acceleration", cov_acceleration, 0.1);
        readParam("cov_bias_gyroscope", cov_bias_gyroscope, 0.1);
        readParam("cov_bias_acceleration", cov_bias_acceleration, 0.1);
        print_table();
        Q.block<3,3>(0,0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3,3>(3,3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3,3>(6,6).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3,3>(9,9).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        SLAM_LOG_INFO << "IESKF noise params gyro=" << cov_gyroscope
                      << ", acc=" << cov_acceleration
                      << ", bg=" << cov_bias_gyroscope
                      << ", ba=" << cov_bias_acceleration;
    }
    IESKF::~IESKF(){}
    void IESKF::predict(IMU imu, double dt){
        imu.acceleration -= X.ba;
        imu.gyroscope -= X.bg;
        auto rotation = X.rotation.toRotationMatrix();
        X.rotation = Eigen::Quaterniond(rotation * so3Exp(imu.gyroscope*dt));
        X.rotation.normalize();
        X.position += X.velocity*dt;
        X.velocity += (rotation*(imu.acceleration)+X.gravity)*dt;

        Eigen::Matrix<double,18,18> Fx;
        Eigen::Matrix<double,18,12> Fw;
        Fx.setIdentity();
        Fw.setZero();

        Fx.block<3,3>(0,0) = so3Exp(-(imu.gyroscope)*dt);
        Fx.block<3,3>(0,9) = -A_T((-imu.gyroscope)*dt)*dt;
        Fx.block<3,3>(6,0) = -rotation * skewSymmetric(imu.acceleration)*dt;
        Fx.block<3,3>(3,6) = Eigen::Matrix3d::Identity()*dt;
        Fx.block<3,3>(6,12) = -rotation*dt;
        Fx.block<3,3>(6,15) = Eigen::Matrix3d::Identity()*dt;

        Fw.block<3,3>(0,0) = -A_T((-imu.gyroscope)*dt)*dt;
        Fw.block<3,3>(6,3) = -rotation*dt;
        Fw.block<3,3>(9,6) = Eigen::Matrix3d::Identity()*dt;
        Fw.block<3,3>(12,9) = Eigen::Matrix3d::Identity()*dt;

        P = Fx*P*Fx.transpose()+Fw*Q*Fw.transpose(); 
    }
    bool IESKF::update(){
        auto x_k_k  = X;
        auto x_k_last = X;
        Eigen::MatrixXd K;
        Eigen::MatrixXd H_k;
        bool converge = true;
        Eigen::Matrix<double, 18, 18> P_in_update;
        int iter_times{10};
        for(int i =0; i<iter_times; ++i){
            auto error_state = getErrorState(x_k_k, X);
            Eigen::Matrix<double, 18, 18> J_inv;
            J_inv.setIdentity();
            J_inv.block<3,3>(0,0) = A_T(error_state.block<3,1>(0,0));
            P_in_update = J_inv * P *J_inv.transpose();
            const auto calc_result = calc_zh_ptr->calculate(x_k_k);
            if(!calc_result.valid){
                return false;
            }
            const Eigen::MatrixXd& z_k = calc_result.Z;
            H_k = calc_result.H;
            Eigen::MatrixXd H_kt = H_k.transpose();
            K = (H_kt * H_k + (P_in_update / 0.001).inverse()).inverse() * H_kt;
            Eigen::Matrix<double,18,1> left  = -K*z_k;
            Eigen::Matrix<double,18,1> right = -(Eigen::Matrix<double,18,18>::Identity()-K*H_k)*J_inv*error_state;
            Eigen::Matrix<double,18,1> update_x = left +right;
            VLOG_EVERY_N(1, 200) << "IESKF update_x frame=" << (cnt_ + 1)
                                 << ", iter=" << (i + 1)
                                 << ", max_abs=" << update_x.cwiseAbs().maxCoeff()
                                 << ", update_x=[" << update_x.transpose() << "]";

            converge = true;
            for (int idx = 0; idx < 18; ++idx) {
                if (std::abs(update_x(idx, 0)) > 0.001) {
                    converge = false;
                    break;
                }
            }
            
            x_k_k.rotation = x_k_k.rotation.toRotationMatrix()*so3Exp(update_x.block<3,1>(0,0));
            x_k_k.rotation.normalize();
            x_k_k.position += update_x.block<3,1>(3,0);
            x_k_k.velocity += update_x.block<3,1>(6,0);
            x_k_k.bg += update_x.block<3,1>(9,0);
            x_k_k.ba += update_x.block<3,1>(12,0);
            x_k_k.gravity += update_x.block<3,1>(15,0);
            if(converge){
                break;
            }
        }
        X = x_k_k;
        P=(Eigen::Matrix<double,18,18>::Identity()-K*H_k)*P_in_update;
        cnt_++;
        if (!converge) {
            SLAM_LOG_WARN << "IESKF update reached max iterations at frame " << cnt_;
        }
        return converge;
    }
    Eigen::Matrix<double, 18, 1> IESKF::getErrorState(const State18& s1, const State18& s2)const{
        Eigen::Matrix<double, 18, 1> es;
        es.block<3,1>(0,0) = SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
        es.block<3,1>(3,0) = s1.position - s2.position;
        es.block<3,1>(6,0) = s1.velocity -s2.velocity;
        es.block<3,1>(9,0) = s1.bg -s2.bg;
        es.block<3,1>(12,0) = s1.ba -s2.ba;
        es.block<3,1>(15,0) = s1.gravity -s2.gravity;
        return es;
    }
    const IESKF::State18& IESKF::getX(){
        return X;
    }
    void IESKF::setX(const State18& x_in){
        X = x_in;
    }
}
