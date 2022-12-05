#ifndef ADWA_UTILS_H
#define ADWA_UTILS_H

#include <Eigen/Geometry>
#include <cmath>

namespace adwa_utils {
    /**
     * @brief Tranform points coordination represented in base frame to world frame.
     * @param Vec3<T> the 3d position of the origin of the base frame w.r.t. world frame;
     * @param Quat<T> the orientation of the base frame (quaternion, wxyz) w.r.t. worlf frame;
     * @param Eigen::Matrix<T, 3, N> The input point(s) is represent in the body frame
     * @return Eigen::Matrix<T, 3, N> The output point(s) is represent in the world frame, but these two point are the same point in physics.
     */
    template<typename T, int N = 1>
    Eigen::Matrix<T, 3, N> invertRigidTransform(Eigen::Matrix<T, 3, 1> t, Eigen::Matrix<T, 4, 1> R, Eigen::Matrix<T, 3, N> points)
    {
        // Eigen::Isometry3f transform; or Eigen::Transform<T,3,Eigen::Isometry>
        Eigen::Transform<T, 3, Eigen::Isometry> transform = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
        transform.translate(t);
        Eigen::Quaternion<T> r{R(0), R(1), R(2), R(3)};
        transform.rotate(r);
        return transform * points;
    }
};

#endif //ADWA_UTILS_H