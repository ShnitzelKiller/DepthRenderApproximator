#ifndef POINTS_FROM_DEPTH_CAMERAUTILS_H
#define POINTS_FROM_DEPTH_CAMERAUTILS_H
#include <Eigen/Core>
#include <iostream>
#include "typedefs.hpp"
#include <random>

namespace geom {

    template <class T>
    /**
     * Generate random angle axis rotation with normally distributed angle and uniformly (on the unit sphere) distributed axis
     * @param stdev standard deviation of angle
     * @param axis output axis
     * @param angle output angle
     */
    void randomAngleAxis(T stdev, Vector3<T> &axis, T &angle) {
        Eigen::Quaternion<T> quat = Eigen::Quaternion<T>::UnitRandom();
        axis[0] = quat.x();
        axis[1] = quat.y();
        axis[2] = quat.z();
        axis.normalize();
        std::normal_distribution<T> distribution(0, stdev);
        static std::default_random_engine generator((unsigned int) time(0));
        angle = distribution(generator);
    }

    template<class T>
    Eigen::Matrix<T, 4, 4> perspective
            (
                    double fovy,
                    double aspect,
                    double zNear,
                    double zFar
            ) {
        typedef Eigen::Matrix<T, 4, 4> Matrix4;

        assert(aspect > 0);
        assert(zFar > zNear);

        double radf = fovy / 180 * M_PI;

        double tanHalfFovy = tan(radf / 2.0);
        Matrix4 res = Matrix4::Zero();
        res(0, 0) = 1.0 / (aspect * tanHalfFovy);
        res(1, 1) = 1.0 / (tanHalfFovy);
        res(2, 2) = -(zFar + zNear) / (zFar - zNear);
        res(3, 2) = -1.0;
        res(2, 3) = -(2.0 * zFar * zNear) / (zFar - zNear);
        return res;
    }

    template<class T>
    Eigen::Matrix<T, 4, 4> lookAt
            (
                    Eigen::Matrix<T, 3, 1> const &eye,
                    Eigen::Matrix<T, 3, 1> const &center,
                    Eigen::Matrix<T, 3, 1> const &up
            ) {
        typedef Eigen::Matrix<T, 4, 4> Matrix4;
        typedef Eigen::Matrix<T, 3, 1> Vector3;

        Vector3 f = (center - eye).normalized();
        Vector3 u = up.normalized();
        Vector3 s = f.cross(u).normalized();
        u = s.cross(f);

        Matrix4 res;
        res << s.x(), u.x(), -f.x(), eye.x(),
	  s.y(), u.y(), -f.y(), eye.y(),
	  s.z(), u.z(), -f.z(), eye.z(),
                0, 0, 0, 1;
        //std::cout << res.inverse() << std::endl;

        return res;
    }
}
#endif //POINTS_FROM_DEPTH_CAMERAUTILS_H
