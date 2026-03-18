#pragma once
#include <Eigen/Dense>
#include <vector>
namespace IESKFSLAM
{
    struct PlaneCheckResult{
        bool valid = false;
        Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    };

    template< typename pointTypeT >
    inline PlaneCheckResult planarCheck(const std::vector<pointTypeT> & points, float threhold){
        Eigen::Vector3d normal_vector;
        Eigen::MatrixXd A;
        Eigen::VectorXd B;
        int point_num = points.size();
        A.resize(point_num,3);
        B.resize(point_num);
        B.setOnes();
        B = -1*B;
        for (int i = 0; i < point_num; i++)
        {
            A(i,0) = points[i].x;
            A(i,1) = points[i].y;
            A(i,2) = points[i].z;
        }

        normal_vector = A.colPivHouseholderQr().solve(B);

        for (int j = 0; j < point_num; j++)
        {
            if (fabs(normal_vector(0) * points[j].x + normal_vector(1) * points[j].y + normal_vector(2) * points[j].z + 1.0f) > threhold)
            {
                return {};
            }
        }
        double normal = normal_vector.norm();
        normal_vector.normalize();
        PlaneCheckResult result;
        result.valid = true;
        result.plane(0) = normal_vector(0);
        result.plane(1) = normal_vector(1);
        result.plane(2) = normal_vector(2);
        result.plane(3) = 1/normal;

        return result;

    }
    template<typename PointType,typename T>
    inline PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t){
        Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
        ep = q*ep+t;
        point.x = ep.x();
        point.y = ep.y();
        point.z = ep.z();
        return point;
    }

}
