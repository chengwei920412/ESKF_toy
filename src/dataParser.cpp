#include "dataParser.h"

DataFiles::DataFiles(std::string path){
    cout << "data path" << path << endl;
    //init files and waste the header line.
    stringstream ssMixed;
    ssMixed << path << "/mixedTimeSeries.txt";
    readerMixed.open(ssMixed.str());
    if (!readerMixed.is_open()){
        cout << "readerMixed open failed: " << endl;
        return;
    }
    char var[256];
    readerMixed.getline(var,sizeof(var),'\n');

    stringstream ssMocap;
    ssMocap << path << "/mocapTimeSeries.txt";
    readerMocap.open(ssMocap.str());
    if (!readerMocap.is_open()){
        cout << "readerMocap open failed: " << endl;
        return;
    }
    readerMocap.getline(var,sizeof(var),'\n');

    stringstream ssIMU;
    ssIMU << path << "/accelGyroTimeSeries.txt";
    readerIMU.open(ssIMU.str());
    if (!readerIMU.is_open()){
        cout << "readerIMU open failed: " << endl;
        return;
    }
    readerIMU.getline(var,sizeof(var),'\n');

    valid = true;
}

int DataFiles::getNext(ifstream &file, mocapData &mocap, imuData &imu, int &type){
    string str;
    std::getline(file, str, ',');
    if (!str.compare("IMU")){
        type = isImuData;

        std::getline(file, str, ',');
        imu.acc[0] = atof(str.c_str());

        std::getline(file, str, ',');
        imu.acc[1] = atof(str.c_str());

        std::getline(file, str, ',');
        imu.acc[2] = atof(str.c_str());

        std::getline(file, str, ',');
        imu.gyr[0] = atof(str.c_str());

        std::getline(file, str, ',');
        imu.gyr[1] = atof(str.c_str());

        std::getline(file, str, ',');
        imu.gyr[2] = atof(str.c_str());

        std::getline(file, str, ',');
        int sec = atoi(str.c_str());

        std::getline(file, str, '\n');
        int nsec = atoi(str.c_str());

        imu.timestamp = sec * 1.0 + nsec * 1.0 * 1e-9;

        return 0;
    }
    else if(!str.compare("Mocap")){
        type = isMocapData;

        std::getline(file,str,',');
        mocap.pos[0] = atof(str.c_str());

        std::getline(file,str,',');
        mocap.pos[1] = atof(str.c_str());

        std::getline(file,str,',');
        mocap.pos[2] = atof(str.c_str());

        std::getline(file,str,',');
        float qx = atof(str.c_str());

        std::getline(file,str,',');
        float qy = atof(str.c_str());

        std::getline(file,str,',');
        float qz = atof(str.c_str());

        std::getline(file,str,',');
        float qw = atof(str.c_str());

        mocap.quat = Eigen::Quaternionf(qw,qx,qy,qz);

        std::getline(file, str, ',');
        int sec = atoi(str.c_str());

        std::getline(file, str, '.');
        int nsec = atoi(str.c_str());

        mocap.timestamp = sec * 1.0 + nsec * 1.0 * 1e-9;

        std::getline(file, str, ',');
        sec = atoi(str.c_str());

        std::getline(file, str, '\n');
        nsec = atoi(str.c_str());

        mocap.receivedTime = sec * 1.0 + nsec * 1.0 * 1e-9;
        return 0;
    }
    file.close();
    return 1;
}

int DataFiles::getNextTimeCorrected(mocapData &mocap, imuData &imu, int &type){
    static mocapData mocapTemp;
    static imuData imuTemp;
    static int primed = 0;
    static int typeTemp;
    int error = 0;
    double offset = 0;
    if (!primed){
        primed = 1;
        error = getNext(readerMocap, mocapTemp, imuTemp, typeTemp);
        error += getNext(readerIMU, mocapTemp, imuTemp, typeTemp);
        double diff = imuTemp.timestamp - mocapTemp.timestamp + offset; // no offset indeed;
        if (diff >= 0){
            typeTemp = isMocapData;
        }
        else{
            typeTemp = isImuData;
        }
        mocap = mocapTemp;
        imu = imuTemp;
        type = typeTemp;
        return error;
    }
    else{
        if (typeTemp == isMocapData){
            error = getNext(readerMocap, mocapTemp, imuTemp, typeTemp);
        }
        else{
            error = getNext(readerIMU, mocapTemp, imuTemp, typeTemp);
        }
        double diff = imuTemp.timestamp - mocapTemp.timestamp + offset;
        if (diff >= 0){
            typeTemp = isMocapData;
        }
        else{
            typeTemp = isImuData;
        }
        mocap = mocapTemp;
        imu = imuTemp;
        type = typeTemp;
        return error;
    }
}