#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#include <iostream>
#include <Eigen/Core>

#define POS_IDX (0)             // 0
#define VEL_IDX (POS_IDX + 3)   // 3
#define QUAT_IDX (VEL_IDX + 3)  // 6
#define BA_IDX (QUAT_IDX + 4)   // 10
#define BG_IDX (BA_IDX + 3)     // 13
#define STATE_SIZE (BG_IDX + 3) // 16

#define dPOS_IDX (0)                // 0
#define dVEL_IDX (dPOS_IDX + 3)     // 3
#define dTHETA_IDX (dVEL_IDX + 3)   // 6
#define dBA_IDX (dTHETA_IDX + 3)    // 9
#define dBG_IDX (dBA_IDX + 3)       // 12
#define dSTATE_SIZE (dBG_IDX + 3)   // 15

#define GRAVITY 9.8

enum dataType{
    isImuData = 0,
    isMocapData = 1
};

struct imuData{
    Eigen::Vector3f acc;
    Eigen::Vector3f gyr;
    double timestamp; // in second
};

struct mocapData{
    Eigen::Vector3f pos;
    Eigen::Quaternionf quat;
    double timestamp; // in second
    double receivedTime; // in second
};

#endif