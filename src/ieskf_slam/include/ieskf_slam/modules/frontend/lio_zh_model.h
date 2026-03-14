
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/base_type.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <memory>
#include "ieskf_slam/math/geometry.h"
#include "ieskf_slam/math/SO3.h"
#ifdef _OPENMP
#include <omp.h>
#endif
namespace IESKFSLAM {
    class LIOZHModel : public IESKF::calcZHInterface{
        private:
        const int NEAR_POINT_NUM{5};
        //1) point_imu 2)normal_vector 3)distance
        using LossType = loss_type<Eigen::Vector3d, Eigen::Vector3d, double>;
        PCLPointCloudPtr current_point_ptr;
        PCLPointCloudConstPtr local_map_ptr;
        KdTreeConstPtr global_map_kdtree_ptr;
        public:
        using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(const PCLPointCloudPtr current_point_ptr, PCLPointCloudConstPtr local_map_ptr, KdTreeConstPtr global_map_kdtree_ptr){
            this->current_point_ptr= current_point_ptr;
            this->local_map_ptr = local_map_ptr;
            this->global_map_kdtree_ptr = global_map_kdtree_ptr;
        }
        bool calculate(const IESKF::State18& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H)override{
            std::vector<LossType> loss_v;
            loss_v.resize(current_point_ptr->size());
            std::vector<LossType> loss_real;
            std::vector<bool> is_effect_points(current_point_ptr->size(), false);
            int vaild_points_num = 0;

            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
                #pragma omp parallel for
            #endif

            for (int i=0; i<current_point_ptr->size(); ++i) {
                Point point_imu = current_point_ptr->points[i];
                Point point_world = IESKFSLAM::transformPoint(point_imu, state.rotation, state.position); //取state计算imu->world
                std::vector<int> point_ind;
                std::vector<float> distance;
                global_map_kdtree_ptr->nearestKSearch(point_world, NEAR_POINT_NUM, point_ind, distance);
                if (point_ind.size() < NEAR_POINT_NUM || distance[NEAR_POINT_NUM-1] > 5.0) {
                    continue;
                }
                std::vector<Point> planar_poionts;
                for(int ni = 0; ni<NEAR_POINT_NUM; ++ni){
                    planar_poionts.push_back(local_map_ptr->at(point_ind[ni]));
                }
                Eigen::Vector4d pabcd;
                if(planarCheck(planar_poionts, pabcd, 0.1)){
                    double pd = pabcd[0]*point_world.x + pabcd[1]*point_world.y + pabcd[2]*point_world.z + pabcd[3];
                    LossType loss;
                    loss.first = {point_imu.x, point_imu.y, point_imu.z};
                    loss.second = {pabcd[0], pabcd[1], pabcd[2]};
                    loss.third = pd;
                    if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
                    double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
                    if(s > 0.9 ){
                        loss_v[i]=loss;
                        is_effect_points[i] = true;
                    }
                }
            }
                for (int i = 0; i < current_point_ptr->size(); ++i) {
                    if (is_effect_points[i]) {
                        loss_real.push_back(loss_v[i]);
                    }
                }
                vaild_points_num = loss_real.size();
                H = Eigen::MatrixXd::Zero(vaild_points_num, 18); 
                Z.resize(vaild_points_num,1);
                for (int vi = 0; vi < vaild_points_num; vi++)
                {
                    // H 记录导数
                    Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(loss_real[vi].first);
                    H.block<1,3>(vi,0) = dr.transpose();
                    H.block<1,3>(vi,3) = loss_real[vi].second.transpose();
                    // Z记录距离
                    Z(vi,0) = loss_real[vi].third;
                }
                return true;
            }
    };
}