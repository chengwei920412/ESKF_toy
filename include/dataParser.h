#ifndef DATA_PARSER_H
#define DATA_PARSER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

#include "common_define.h"

using namespace std;

class DataFiles{
public:
    DataFiles(std::string path);
    int getNextTimeCorrected(mocapData &mocap, imuData &imu, int &type);
    bool getDataFileValid(){
        return valid;
    }

private:
    ifstream readerMixed;
    ifstream readerIMU;
    ifstream readerMocap;
    bool valid = false;
    int getNext(ifstream &file, mocapData &mocap, imuData &imu, int &type);
};

#endif