#pragma once
#include <iostream>

namespace IESKFSLAM{
    class Timestamp{
        private:
            uint64_t nsec_;
            double sec_;
        public:
            Timestamp(uint64_t nsec = 0){
                nsec_ = nsec;
                sec_ = static_cast<double>(nsec_) / 1e9;
            }
            void fromSec(double sec){
                sec_ = sec;
                nsec_ = static_cast<uint64_t>(sec_ * 1e9);
            }
            void fromNSec(uint64_t nsec){
                nsec_ = nsec;
                sec_ = static_cast<double>(nsec_) / 1e9;
            }
            void clear (){
                nsec_ = 0;
                sec_ = 0.0;
            }
            const double& sec() const{
                return sec_;
            }
            const uint64_t& nsec() const{
                return nsec_;
            }
    };
}