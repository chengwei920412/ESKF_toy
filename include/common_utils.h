#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

// the quaternion format conversion between Hamilton convention (wxyz) and Eigen convention (xyzw)
Eigen::Quaternionf HamiltonToQuat(const Eigen::Vector4f &hamilton);

Eigen::Vector4f QuatToHamilton(const Eigen::Quaternionf &quat);

// skew symetric of a 3-vec
Eigen::Matrix3f getSkew(const Eigen::Vector3f &vec);

Eigen::Quaternionf rotVecToQuat(const Eigen::Vector3f &rotVec);

Eigen::Vector3f QuatToRotVec(const Eigen::Quaternionf & quat);

Eigen::Matrix3f rotVecToMat(const Eigen::Vector3f &rotVec);

Eigen::Vector3f MatToRotVec(const Eigen::Matrix3f & mat);

// P62, Equ.280
// not used in the implementation
Eigen::Matrix<float, 4, 3> getQ_dtheta(const Eigen::Quaternionf & quat);

#endif