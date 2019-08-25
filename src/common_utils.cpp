#include "common_utils.h"

// the quaternion format conversion between Hamilton convention (wxyz) and Eigen convention (xyzw)
Eigen::Quaternionf HamiltonToQuat(const Eigen::Vector4f &hamilton){
    Eigen::Vector4f vec;
    vec << hamilton.block<3, 1>(1, 0),      // x, y, z
                hamilton.block<1, 1>(0, 0);     // w
    return Eigen::Quaternionf(vec);
}

Eigen::Vector4f QuatToHamilton(const Eigen::Quaternionf &quat){
    Eigen::Vector4f vec;
    vec << quat.coeffs().block<1, 1>(3, 0),         // w
            quat.coeffs().block<3, 1>(0, 0);        // x, y, z
    return vec;
}

// skew symetric of a 3-vec
Eigen::Matrix3f getSkew(const Eigen::Vector3f &vec){
    Eigen::Matrix3f mat;
    mat << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return mat;
}

Eigen::Quaternionf rotVecToQuat(const Eigen::Vector3f &rotVec){
    float angle = rotVec.norm();
    Eigen::Vector3f axis = (angle == 0) ? Eigen::Vector3f(0, 0, 0) : rotVec.normalized();
    return Eigen::Quaternionf(Eigen::AngleAxisf(angle, axis));
}

Eigen::Vector3f QuatToRotVec(const Eigen::Quaternionf & quat){
    Eigen::AngleAxisf angAx(quat);
    return angAx.angle() * angAx.axis();
}

// Eigen::Matrix3f rotVecToMat(const Eigen::Vector3f &rotVec){
//     float angle = rotVec.norm();
//     Eigen::Vector3f axis = (angle == 0) ? Eigen::Vector3f(0, 0, 0) : rotVec.normalized();
//     Eigen::AngleAxisf angAx(angle, axis);
//     return angAx.toRotationMatrix();
// }

Eigen::Vector3f MatToRotVec(const Eigen::Matrix3f & mat){
    Eigen::AngleAxisf angAx(mat);
    return angAx.angle() * angAx.axis();
}

// P62, Equ.280
// not used in the implementation
Eigen::Matrix<float, 4, 3> getQ_dtheta(const Eigen::Quaternionf & quat){
    Eigen::Vector4f q = quat.coeffs();
    float qx = q(0);
    float qy = q(1);
    float qz = q(2);
    float qw = q(3);
    Eigen::Matrix<float, 4, 3> Q_dTheta;
    Q_dTheta << -qx, -qy, -qz,
                qw, -qz, qy,
                qz, qw, -qx,
                -qy, qx, qw;
    return Q_dTheta;
}