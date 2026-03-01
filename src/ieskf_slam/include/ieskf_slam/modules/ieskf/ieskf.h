#pragma once

#include "ieskf_slam/modules/module_base.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include "ieskf_slam/type/imu.h"
namespace IESKFSLAM{
    class IESKF: private ModuleBase{
        public:
            using Ptr = std::shared_ptr<IESKF>;
            struct State18{
                Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
                Eigen::Vector3d position = Eigen::Vector3d::Zero();
                Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
                Eigen::Vector3d bg = Eigen::Vector3d::Zero();
                Eigen::Vector3d ba = Eigen::Vector3d::Zero();
                Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
            };
        private:
            State18 X;
            Eigen::Matrix<double,18,18> P;
            Eigen::Matrix<double,12,12> Q;
        public:
            IESKF(const std::string&config_path,const std::string&prefix);
            ~IESKF();
            void predict(IMU& imu, double dt);
            bool update();
            const State18& getX();
            void setX(const State18& x_in);
            
    };
}