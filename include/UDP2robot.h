//
// Created by root on 25/09/18.
//

#ifndef ORB_SLAM2_UDP2ROBOT_H
#define ORB_SLAM2_UDP2ROBOT_H

//
// Created by skaegy on 31/08/18.
//

#ifndef RGBDSTREAMING_UDPSOCKETSIMPLE_H
#define RGBDSTREAMING_UDPSOCKETSIMPLE_H

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <mutex>
#include <cstring>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

namespace ORB_SLAM2{
class System;
class Viewer;

class udpSocket{

public:
    udpSocket(const string strSettingFile);

    void RunServer();

    void RunClient();

    void SetViewer(Viewer *pViewer);

    bool mCloseServer = false;
    bool mCloseClient = false;
    bool isServerOk, isClientOk;

    std::vector <int> mControlCommand;

private:
    bool CreateServerSocket();

    bool CreateClientSocket();

    int GenerateRotCmd(cv::Vec3f HIP_C, double thres);

    int GenerateForwardControlCmd(cv::Vec3f HIP_C, double angleThres, double distThresMin, double distThresMax);

    int GenerateBackwardControlCmd(cv::Vec3f HIP_C, double angleThres, double distThresMin, double distThresMax);

    // Parameters for udp
    struct sockaddr_in addrServer, addrClient;
    int mPortIn, mPortOut;
    int mReceiverInterval, mSenderInterval;
    const char* mClientIP;
    int mTimeOutMax;
    int mSockfdServer, mSockfdClient;
    int mBuffersize = 1000;
    char mbuffer[1000];

    // Parameters for double robot control
    int mRobotMode;
    double mDistThresMin, mDistThresMax;
    double mAngleThres;

protected:
    Viewer* mpViewer;

};

}
#endif //RGBDSTREAMING_UDPSOCKETSIMPLE_H


#endif //ORB_SLAM2_UDP2ROBOT_H
