#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_
#include <Eigen/Dense>
template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;
template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

#endif