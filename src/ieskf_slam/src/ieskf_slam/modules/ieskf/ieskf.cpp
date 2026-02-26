#include"ieskf_slam/modules/ieskf/ieskf.h"

namespace IESKFSLAM{
    IESKF::IESKF(const std::string&config_path,const std::string&prefix): ModuleBase(config_path, prefix, "IESKF"){}
    IESKF::~IESKF(){}
    void IESKF::predict(const IMU &imu, double dt){}
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