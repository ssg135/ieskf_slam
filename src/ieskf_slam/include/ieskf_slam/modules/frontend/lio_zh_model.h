
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/modules/map/map_manager_base.h"
#include <Eigen/src/Core/Matrix.h>
#include <cstdint>
#include <cmath>
#include <memory>
#include <vector>
#include "ieskf_slam/math/geometry.h"
#include "ieskf_slam/math/SO3.h"
#if defined(MP_EN) && defined(_OPENMP)
#include <omp.h>
#endif
namespace IESKFSLAM {
    class LIOZHModel : public IESKF::calcZHInterface{
        private:
        const int NEAR_POINT_NUM{5};
        //1) point_imu 2)normal_vector 3)distance
        using LossType = loss_type<Eigen::Vector3d, Eigen::Vector3d, double>;
        PCLPointCloudPtr current_point_ptr;
        MapManagerBase::ConstPtr map_manager_ptr;
        mutable std::vector<std::vector<Point>> nearest_points_cache_;
        mutable std::vector<std::uint8_t> nearest_points_valid_;

        void ensureFrameCache(std::size_t point_count) const {
            if (nearest_points_cache_.size() != point_count) {
                nearest_points_cache_.assign(point_count, {});
                nearest_points_valid_.assign(point_count, 0);
            }
        }
        public:
        using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(const PCLPointCloudPtr current_point_ptr, const MapManagerBase::ConstPtr& map_manager_ptr){
            this->current_point_ptr= current_point_ptr;
            this->map_manager_ptr = map_manager_ptr;
        }
        void resetFrameCache(){
            nearest_points_cache_.clear();
            nearest_points_valid_.clear();
        }
        IESKF::CalcZHResult calculate(const IESKF::State18& state, bool need_rematch) const override{
            if (!current_point_ptr || !map_manager_ptr || current_point_ptr->empty()) {
                return {};
            }
            ensureFrameCache(current_point_ptr->size());
            std::vector<LossType> loss_v;
            loss_v.resize(current_point_ptr->size());
            std::vector<LossType> loss_real;
            std::vector<bool> is_effect_points(current_point_ptr->size(), false);
            int vaild_points_num = 0;

            #if defined(MP_EN) && defined(_OPENMP)
                omp_set_num_threads(MP_PROC_NUM);
                #pragma omp parallel for
            #endif

            for (int i=0; i<current_point_ptr->size(); ++i) {
                Point point_imu = current_point_ptr->points[i];
                Point point_world = IESKFSLAM::transformPoint(point_imu, state.rotation, state.position); //取state计算imu->world
                const std::vector<Point>* nearest_points_ptr = nullptr;
                if (need_rematch || !nearest_points_valid_[i]) {
                    std::vector<Point> nearest_points;
                    std::vector<float> distance;
                    if (!map_manager_ptr->nearestKSearch(point_world, NEAR_POINT_NUM, nearest_points, distance) ||
                        nearest_points.size() < NEAR_POINT_NUM || distance[NEAR_POINT_NUM-1] > 5.0) {
                        nearest_points_valid_[i] = 0;
                        nearest_points_cache_[i].clear();
                        continue;
                    }
                    nearest_points_cache_[i] = std::move(nearest_points);
                    nearest_points_valid_[i] = 1;
                }
                if (!nearest_points_valid_[i]) {
                    continue;
                }
                nearest_points_ptr = &nearest_points_cache_[i];
                const auto plane_result = planarCheck(*nearest_points_ptr, 0.1);
                if(plane_result.valid){
                    const Eigen::Vector4d& pabcd = plane_result.plane;
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
                if(vaild_points_num == 0){
                    return {};
                }
                IESKF::CalcZHResult result;
                result.valid = true;
                result.H = Eigen::MatrixXd::Zero(vaild_points_num, 18); 
                result.Z.resize(vaild_points_num,1);
                for (int vi = 0; vi < vaild_points_num; vi++)
                {
                    // H 记录导数
                    Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(loss_real[vi].first);
                    result.H.block<1,3>(vi,0) = dr.transpose();
                    result.H.block<1,3>(vi,3) = loss_real[vi].second.transpose();
                    // Z记录距离
                    result.Z(vi,0) = loss_real[vi].third;
                }
                return result;
            }
    };
}
