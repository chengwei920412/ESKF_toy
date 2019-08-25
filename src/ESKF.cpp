#include "ESKF.h"

ESKF::ESKF(Eigen::Vector3f gravity,
    const Eigen::Matrix<float, STATE_SIZE, 1> &initialState,
    const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> &initialP,
    float var_acc, float var_omega, float var_acc_bias, float var_omega_bias,
    string output_path)
    : var_acc_(var_acc),
    var_omega_(var_omega),
    var_acc_bias_(var_acc_bias),
    var_omega_bias_(var_omega_bias),
    gravity_(gravity),
    norminalState_(initialState),
    P_(initialP){
        // set errorState to all zeros;
        errorState_.setZero();

        // set constant items of state Jacobian, P59, Equ. 269
        F_x_.setZero();            
        F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX) = Eigen::Matrix3f::Identity(); // dPos row            
        F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX) = Eigen::Matrix3f::Identity(); // dVel row
        F_x_.block<3, 3>(dBA_IDX, dBA_IDX) = Eigen::Matrix3f::Identity();   // dBa row            
        F_x_.block<3, 3>(dBG_IDX, dBG_IDX) = Eigen::Matrix3f::Identity();   // dBg row

        // set noise Jacobian, P60, Equ.270, won't be used actually
        F_i_.setZero();
        F_i_.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity();   // random acc noise
        F_i_.block<3, 3>(6, 3) = Eigen::Matrix3f::Identity();   // random gyro noise
        F_i_.block<3, 3>(9, 6) = Eigen::Matrix3f::Identity();   // random acc bias noise
        F_i_.block<3, 3>(12, 9) = Eigen::Matrix3f::Identity();  // random gyro bias noise

        // open the trajectory output file
        out_path = output_path;
        of.open(out_path + "out_traj.txt");
        if (!of.is_open()){
            cout << "output file open failed!" << endl;
            return;
        }
        // of << "timestamp tx ty tz qx qy qz qw" << endl;

        ref_of.open(out_path + "ref_traj.txt");
        if (!ref_of.is_open()){
            cout << "output ref file open failed!" << endl;
            return;
        }
        // ref_of << "timestamp tx ty tz qx qy qz qw" << endl;

        cout << "ESKF construction: Success!" << endl;
}

void ESKF::predictIMU(const imuData &imu, const double &dt){
    // cout.precision(13);
    // cout << imu.timestamp << " ";
    // cout.precision(6);
    // cout << imu.acc(0) << " " << imu.acc(1) << " " << imu.acc(2) << " " << imu.gyr(0) << " " << imu.gyr(1) << " " << imu.gyr(2) << endl;
    // cout << "diff t: " << dt << endl;
    // access the norminal state
    // Eigen::Vector3f position = norminalState_.block<3, 1>(POS_IDX, 0);
    Eigen::Vector3f velocity = norminalState_.block<3, 1>(VEL_IDX, 0);
    Eigen::Vector4f orientation = norminalState_.block<4, 1>(QUAT_IDX, 0); // Hamilton convention, wxyz
    Eigen::Quaternionf quat = HamiltonToQuat(orientation);
    Eigen::Vector3f bias_acc = norminalState_.block<3, 1>(BA_IDX, 0);
    Eigen::Vector3f bias_gyr = norminalState_.block<3, 1>(BG_IDX, 0);

    // precompute auxiliary
    Eigen::Vector3f acc_correct = imu.acc - bias_acc;
    // cout << "acc_correct: " << acc_correct.transpose() << endl;
    Eigen::Matrix3f Rot = quat.matrix(); // Rwi??
    // cout << "rot: " << endl;
    // cout << Rot << endl;
    Eigen::Vector3f acc_correct_world = Rot * acc_correct;
    // cout << "acc_correct_to_world: " << acc_correct_world.transpose() << endl;
    Eigen::Vector3f acc_correct_world_gravity = acc_correct_world + gravity_;
    // cout << "acc_correct_world_gravity: " << acc_correct_world_gravity.transpose() << endl;
    Eigen::Vector3f omega_correct = imu.gyr - bias_gyr;

    //norminal state propagation, P58, Equ.259
    norminalState_.block<3, 1>(POS_IDX, 0) += velocity * dt + 0.5f * acc_correct_world_gravity * dt * dt;
    norminalState_.block<3, 1>(VEL_IDX, 0) += acc_correct_world_gravity * dt;
    norminalState_.block<4, 1>(QUAT_IDX, 0) = QuatToHamilton(quat * rotVecToQuat(omega_correct * dt)).normalized();
    // norminalState_.block<3, 1>(BA_IDX, 0) = bias_acc;     // unchanged
    // norminalState_.block<3, 1>(BG_IDX, 0) = bias_gyr;     // unchanged
    // cout << "omega_correct: " << omega_correct.transpose() << endl;
    // cout << "predict: " << norminalState_.block<7, 1>(POS_IDX, 0).transpose() << endl;

    // error state propagation, P58, Equ.260
    /* propagation of error state will always be zeros actually, so these can be ignored in implementation
    Eigen::Vector3f dpos = errorState.block<3, 1>(dPOS_IDX, 0);
    Eigen::Vector3f dVel = errorState.block<3, 1>(dVEL_IDX, 0);
    Eigen::Vector3f dtheta = errorState.block<3, 1>(dTHETA_IDX, 0);
    Eigen::Vector3f dba = errorState.block<3, 1>(dBA_IDX, 0);
    Eigen::Vector3f dbg = errorState.block<3, 1>(dBG_IDX, 0);

    errorState.block<3, 1>(dPOS_IDX, 0) += dvel * dt;
    errorState.block<3, 1>(dVEL_IDX, 0) += (-Rot * getSkew(acc_correct) * dtheta - Rot * dba) * dt;
    errorState.block<3, 1>(dTHETA_IDX, 0) = rotVecToQuat(omega_correct * dt).toRotationMatrix().transpose() * dtheta - dbg * dt;
    errorState.block<3, 1>(dBA_IDX, 0) = dba;
    errorState.block<3, 1>(dBG_IDX, 0) = dbg;
    */

   // Fx, the dynamic block, P59, Equ.269
   // dPos row
   // F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX) = Eigen::Matrix3f.Identity() * dt;
   F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt);
   // dVel row
   F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = - Rot * getSkew(acc_correct) * dt;
   F_x_.block<3, 3>(dVEL_IDX, dBA_IDX) = - Rot * dt; 
   // dTheta row
   F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = rotVecToQuat(omega_correct * dt).toRotationMatrix().transpose();
   // F_x_.block<3, 3>(dTHETA_IDX, dBG_IDX) = - Eigen::Matrix3f.Identity() * dt;
   F_x_.block<3, 3>(dTHETA_IDX, dBG_IDX).diagonal().fill(-dt);

   // P propagation, P59, Equ.268
   P_ = F_x_ * P_ * F_x_.transpose();
   P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * dt * dt;
   P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * dt * dt;
   P_.diagonal().block<3, 1>(dBA_IDX, 0).array() += var_acc_bias_ * dt;
   P_.diagonal().block<3, 1>(dBG_IDX, 0).array() += var_omega_bias_ * dt;    
}

void ESKF::updateMocap(const mocapData &mocap, const Eigen::Matrix3f & SIGMA_P, const Eigen::Matrix3f &SIGMA_Q){
    // compute the error observations for dPos and dTheta
    // the error observations can be directly computed by 'minus' between state observation and norminal state;
    Eigen::Vector3f delta_pos = mocap.pos - norminalState_.block<3, 1>(POS_IDX, 0);
    Eigen::Quaternionf q_meas = mocap.quat;
    Eigen::Quaternionf q_norminal = HamiltonToQuat(norminalState_.block<4, 1>(QUAT_IDX, 0));
    Eigen::Quaternionf q_delta = q_norminal.conjugate() * q_meas;
    Eigen::Vector3f delta_theta = QuatToRotVec(q_delta);

    Eigen::Matrix<float, 6, 1> delta_meas;
    delta_meas << delta_pos, delta_theta;

    // compute H, the above operation transform the state observation to error state observation, P61, Equ.277
    // so here H is \partial\delta x / \partial\delta x, identity;
    Eigen::Matrix<float, 6, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dPOS_IDX) = Eigen::Matrix3f::Identity();
    H.block<3, 3>(3, dTHETA_IDX) = Eigen::Matrix3f::Identity();

    // compute V, covariance of the measurement, P61, Equ.272
    Eigen::Matrix<float, 6, 6> V;
    V.block<3, 3>(0, 0) = SIGMA_P;
    V.block<3, 3>(3, 3) = SIGMA_Q;

    // Kalman gain, P61, Equ.273
    Eigen::Matrix<float, dSTATE_SIZE, 6> PHt = P_ * H.transpose();
    Eigen::Matrix<float, dSTATE_SIZE, 6> K = PHt * (H * PHt + V).inverse();

    // update error state, P61, Equ.274
    errorState_ = K * delta_meas;
    
    // and covariance 
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * V * K.transpose(); //(Joseph form: P61, note)
    // P_ = I_KH * P_; P_ = 0.5 * (P_ + P_.transpose()); // P61, Equ.275

    // injection of error state, P62, Equ.281/282
    norminalState_.block<3, 1>(POS_IDX, 0) += errorState_.block<3, 1>(dPOS_IDX, 0);
    norminalState_.block<3, 1>(VEL_IDX, 0) += errorState_.block<3, 1>(dVEL_IDX, 0);
    Eigen::Vector4f orientation = norminalState_.block<4, 1>(QUAT_IDX, 0);
    Eigen::Quaternionf quat = HamiltonToQuat(orientation);
    norminalState_.block<4, 1>(QUAT_IDX, 0) = QuatToHamilton(quat * rotVecToQuat(errorState_.block<3, 1>(dTHETA_IDX, 0))).normalized();
    norminalState_.block<3, 1>(BA_IDX, 0) += errorState_.block<3, 1>(dBA_IDX, 0);
    norminalState_.block<3, 1>(BG_IDX, 0) += errorState_.block<3, 1>(dBG_IDX, 0);

    // reset error state and covariance (P63, Equ.285, Equ.287)
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> G;    
    G.setIdentity();
    G.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = Eigen::Matrix3f::Identity() - getSkew(0.5f * errorState_.block<3, 1>(dTHETA_IDX, 0));
    P_ = G * P_ * G.transpose();   
    errorState_.setZero(); 

    // ouptut the norminal state and ref mocap state
    {
        if (!of.is_open()){
            cout << "trajectory output file not opened!" << endl;
            return;
        }
        Eigen::Vector3f pos = norminalState_.block<3, 1>(POS_IDX, 0);
        Eigen::Vector4f orientation = norminalState_.block<4, 1>(QUAT_IDX, 0);
        of.precision(13);
        of << mocap.timestamp << " ";
        of.precision(8);
        of << pos(0) << " " << pos(1) << " " << pos(2) << " " 
            << orientation(1) << " " << orientation(2) << " " << orientation(3) << " " << orientation(0) << endl;

        if (!ref_of.is_open()){
            cout << "ref trajectory output file not opened!" << endl;
            return;
        }
        ref_of.precision(13);
        ref_of << mocap.timestamp << " ";
        ref_of.precision(8);
        ref_of << mocap.pos(0) << " " << mocap.pos(1) << " " << mocap.pos(2) << " "
                << mocap.quat.x() << " " << mocap.quat.y() << " " << mocap.quat.z() << " " << mocap.quat.w() << endl;
    }
}


// just stack all states into a long vector
Eigen::Matrix<float, STATE_SIZE, 1> ESKF::makeState(
    const Eigen::Vector3f &p,
    const Eigen::Vector3f &v,
    const Eigen::Quaternionf &q,
    const Eigen::Vector3f &ba,
    const Eigen::Vector3f &bg){
    Eigen::Matrix<float, STATE_SIZE, 1> mat;
    mat << p, v, QuatToHamilton(q).normalized(), ba, bg;
    cout << "makeState: construction of the init state" << endl;
    cout << "init state: " << endl;
    cout << mat.transpose() << endl;
    return mat;
}

// just put the covariance of each error state into the corresponding position in the P
Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> ESKF::makeP(
    const Eigen::Matrix3f &cov_dPos,
    const Eigen::Matrix3f &cov_dVel,
    const Eigen::Matrix3f &cov_dTheta,
    const Eigen::Matrix3f &cov_dBa,
    const Eigen::Matrix3f &cov_dBg){
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> mat;
    mat.setZero();
    mat.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_dPos;
    mat.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_dVel;
    mat.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dTheta;
    mat.block<3, 3>(dBA_IDX, dBA_IDX) = cov_dBa;
    mat.block<3, 3>(dBG_IDX, dBG_IDX) = cov_dBg;
    cout << "makeP: construction of the initial covariance" << endl;
    cout << "init P: " << endl;
    cout << mat << endl;
    return mat;   
}