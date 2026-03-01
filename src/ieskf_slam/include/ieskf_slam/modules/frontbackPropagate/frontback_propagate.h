#pragma once

#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/measure_group.h"
#include <memory>

namespace IESKFSLAM {
    class FrontbackPropagate{
        private:
            
        public:
            using Ptr = std::shared_ptr<FrontbackPropagate>;
            IMU last_imu;
            double imu_scale;
            FrontbackPropagate();
            ~FrontbackPropagate();
            void propagate(MeasureGroup& mg, IESKF::Ptr ieskf_ptr);
    }; 
}
