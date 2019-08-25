#include <iostream>
#include <math.h>
#include "ESKF.h"
#include "dataParser.h"

using namespace std;

// the noise sigma
float sigma_acc = 0.00124;  // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
float sigma_gyro = 0.276;   // [rad/s] (value derived from Noise Spectral Density in datasheet)
float sigma_acc_drift = 0.001f * sigma_acc;     // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
float sigma_gyro_drift = 0.001f * sigma_gyro;   // [rad/s sqrt(s)] (Educated guess, real value to be measured)

// the error state sigma
float sigma_init_dPos = 1.0f;                       // [m]
float sigma_init_dVel = 0.1f;                       // [m/s]
float sigma_init_dtheta = 1.0f;                     // [rad]
float sigma_init_acc_bias = 10 * sigma_acc_drift;   // [m/s^2]
float sigma_init_gyro_bias = 10 * sigma_gyro_drift; // [rad/s]

// variance of the measurement of mocap
float sigma_mocap_pos = 0.003;  // [m]
float sigma_mocap_rot = 0.03;   // [rad]

// init states
Eigen::Vector3f init_pos(0, 0, 1);
Eigen::Vector3f init_vel(0, 0, 0);
Eigen::Quaternionf init_quat(Eigen::AngleAxisf(0.0f, Eigen::Vector3f(0, 0, 1)));
Eigen::Vector3f init_acc_bias(0,0,0);//(-1.26, -1.09, -1.977);
Eigen::Vector3f init_gyr_bias(0,0,0);//(0.144, -0.01, 0);
Eigen::Vector3f init_gravity(0, 0, -GRAVITY);

int main(int argc, char **argv){
    if (argc != 3){
        cout << "Usage: dataset_path output_path" << endl;
        return 0;
    }
    char * ch = argv[1];
    string dataPath(ch);
    DataFiles filesObj(dataPath);
    if (!filesObj.getDataFileValid()){
        cout << dataPath << " open failed!!" << endl;
        return 0;
    }

    cout << "init quaternion: " << init_quat.x() << " " << init_quat.y() << " " << init_quat.z() << " " << init_quat.w() << endl;
    cout << "init quaternion: " << init_quat.coeffs().block<4, 1>(0,0) << endl;
    ch = argv[2];
    string outPath(ch);
    // eskfSpoof - Will be fed data as if there is no lag, this is the comparison case
    ESKF eskfSpoof(
        init_gravity, 
        ESKF::makeState(init_pos, init_vel, init_quat, init_acc_bias, init_gyr_bias),
        ESKF::makeP(
            sigma_init_dPos * sigma_init_dPos * Eigen::Matrix3f::Identity(),
            sigma_init_dVel * sigma_init_dVel * Eigen::Matrix3f::Identity(),
            sigma_init_dtheta * sigma_init_dtheta * Eigen::Matrix3f::Identity(),
            sigma_init_acc_bias * sigma_init_acc_bias * Eigen::Matrix3f::Identity(),
            sigma_init_gyro_bias * sigma_init_gyro_bias * Eigen::Matrix3f::Identity()
        ),
        sigma_acc * sigma_acc,
        sigma_gyro * sigma_gyro,
        sigma_acc_drift * sigma_acc_drift,
        sigma_gyro_drift * sigma_gyro_drift,
        outPath
    );

    bool flag = false;
    static int mocap_cnt = 0;
    double last_t = 0;
    double diff_t = 0;
    imuData last_imu;
    while (!flag && mocap_cnt < 10000){
        imuData imu;
        mocapData mocap;
        int type;
        flag = filesObj.getNextTimeCorrected(mocap, imu, type);
        if (type == isImuData){ 
            diff_t = imu.timestamp - last_t;
            // cout << "imu diff " << diff_t << endl;
            last_t = imu.timestamp;
            if (diff_t > 1999){
                diff_t = 0.001;
            }
            eskfSpoof.predictIMU(imu, diff_t);
            last_imu = imu;
        }
        else{ // is mocap data
            mocap_cnt++;
            diff_t = mocap.timestamp - last_t;
            last_t = mocap.timestamp;
            // cout << "mocap diff " << diff_t << endl;
            if (diff_t > 0.0001){
                eskfSpoof.predictIMU(last_imu, diff_t); // // should let imu come first, otherwise, the last_imu will be invalid
            }
            eskfSpoof.updateMocap(mocap, 
                                sigma_mocap_pos * sigma_mocap_pos * Eigen::Matrix3f::Identity(), 
                                sigma_mocap_rot * sigma_mocap_rot * Eigen::Matrix3f::Identity());
        }
    }

    return 1;
}