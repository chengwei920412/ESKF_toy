#ifndef ESKF_H
#define ESKF_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#include "common_define.h"
#include "common_utils.h"

using namespace std;

// the main ESKF class
class ESKF{
public:
    ESKF() {};

    ESKF(Eigen::Vector3f gravity,
        const Eigen::Matrix<float, STATE_SIZE, 1> &initialState,
        const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> &initialP,
        float var_acc, float var_omega, float var_acc_bias, float var_omega_bias,
        string output_traj_file);

    ~ESKF(){
        if (of.is_open()){
            of.close();
        }
    };

    void predictIMU(const imuData &imu, const double & dt);
    void updateMocap(const mocapData &mocap, const Eigen::Matrix3f & SIGMA_P, const Eigen::Matrix3f &SIGMA_Q);

    // Concatenates relevant vectors to one large vector.
    static Eigen::Matrix<float, STATE_SIZE, 1> makeState(
        const Eigen::Vector3f &p,
        const Eigen::Vector3f &v,
        const Eigen::Quaternionf &q,
        const Eigen::Vector3f &ba,
        const Eigen::Vector3f &bg);

    // Inserts relevant parts of the block-diagonal of the P matrix
    static Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> makeP(
        const Eigen::Matrix3f &cov_dPos,
        const Eigen::Matrix3f &cov_dVel,
        const Eigen::Matrix3f &cov_dTheta,
        const Eigen::Matrix3f &cov_dBa,
        const Eigen::Matrix3f &cov_dBg);
    
private:
    // IMU noise, used in prediction
    float var_acc_;
    float var_omega_;
    float var_acc_bias_;
    float var_omega_bias_;

    // given the gravity
    Eigen::Vector3f gravity_;
    
    // norminal state
    Eigen::Matrix<float, STATE_SIZE, 1> norminalState_; // transform IMU to world, quaternion in Hamilton convention
    
    // error state and cov
    Eigen::Matrix<float, dSTATE_SIZE, 1> errorState_;
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P_;

    // Jacobian of the state transition (for error state and error state Cov), 
    // ref => P59, Equ.269, gravity is ignored
    // static items are precomputed
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> F_x_;

    // Jacobian of the noise, identity acctually, written here but not operated
    Eigen::Matrix<float, dSTATE_SIZE, 12> F_i_;

    // output
    string out_path;
    ofstream of;
    ofstream ref_of;
};

#endif