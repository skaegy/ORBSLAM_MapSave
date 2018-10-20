//
// Created by root on 25/09/18.
//
//
// Created by root on 31/08/18.
//
#include "UDP2robot.h"
#include <string>
#include <thread>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <include/Viewer.h>

using namespace std;

namespace ORB_SLAM2
{

udpSocket::udpSocket(const string strSettingFile)
{
    cv::FileStorage fs(strSettingFile, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "Failed to open setting file! Exit!" << std::endl;
        exit(-1);
    }

    mPortIn = fs["Port_in"];
    mPortOut = fs["Port_out"];
    mSenderInterval = fs["Send_inverval"];
    mReceiverInterval = fs["Receiver_interval"];
    const string server_addr = fs["IP_client"];
    mClientIP = server_addr.c_str();
    mTimeOutMax = fs["timeout_max"];
    mRobotMode = fs["Robot_mode"];
    mAngleThres = fs["AngleThres"];
    mDistThresMax = fs["DistThresMax"];
    mDistThresMin = fs["DistThresMin"];

    CreateServerSocket();
    //CreateClientSocket();

}

void udpSocket::RunServer()
{
    //int counter = 0;
    socklen_t  addr_len=sizeof(addrServer);
    while(1){
        cv::Vec3f HIP_C = mpViewer->mHIP_C;
        int cmd = 0;
        if (mRobotMode == 0)
            cmd = GenerateForwardControlCmd(HIP_C, mAngleThres, mDistThresMin, mDistThresMax);
        else if (mRobotMode == 1)
            cmd = GenerateBackwardControlCmd(HIP_C, mAngleThres, mDistThresMin, mDistThresMax);

        //cmd = GenerateRotCmd(HIP_C, mAngleThres);
        std::string send_str = std::to_string(cmd);

        sendto(mSockfdServer, send_str.c_str(), send_str.length(), 0, (sockaddr*)&addrServer, addr_len);
        //printf("Server: Sended %d\n", ++counter);
        usleep(mSenderInterval*1e3);

        if (mCloseServer)
            break;
    }
    puts("Server is closed!");
    close(mSockfdServer);
}

void udpSocket::RunClient()
{
    int counter = 0, timeout_cnt = 0;

    while(1){
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));

        int sz = recvfrom(mSockfdClient, mbuffer, mBuffersize, 0, (sockaddr*)&src, &src_len);
        if (sz > 0){
            mbuffer[sz] = 0;
            printf("Get Message %d: %s\n", counter++, mbuffer);
            timeout_cnt = 0;
        }
        else{
            timeout_cnt++;
            printf("No data: %d\n", timeout_cnt);
            if (timeout_cnt > mTimeOutMax)
                break;
        }

        if (mCloseClient){
            close(mSockfdClient);
            break;
        }
    }
    printf("No data is received from port: %d \n", mPortOut);
    close(mSockfdClient);
}

bool udpSocket::CreateServerSocket(){
    // create socket
    mSockfdServer = socket(AF_INET, SOCK_DGRAM, 0);
    if (mSockfdServer == -1){
        puts("Server: Failed to create server socket");
        exit(-1);
    }
    // Set IP address and port
    //socklen_t  addr_len=sizeof(addrServer);
    memset(&addrServer, 0, sizeof(addrServer));

    addrServer.sin_family = AF_INET;
    addrServer.sin_port   = htons(mPortOut);
    addrServer.sin_addr.s_addr = inet_addr(mClientIP);

    cout << endl;
    puts("Server is established!");
    printf("Listen to the port %s\n", mClientIP);
    return true;
}

bool udpSocket::CreateClientSocket(){
    // create socket
    mSockfdClient = socket(AF_INET, SOCK_DGRAM, 0);
    if(-1==mSockfdClient){
        puts("Client: Failed to create socket");
        exit(-1);
    }
    // Set IP address and port
    socklen_t          addr_len=sizeof(addrClient);

    memset(&addrClient, 0, sizeof(addrClient));
    addrClient.sin_family = AF_INET;          // Use IPV4
    addrClient.sin_port   = htons(mPortOut);
    addrClient.sin_addr.s_addr = htonl(INADDR_ANY);

    // set listen interval
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = mReceiverInterval*1000;  // millisecond to usecond
    setsockopt(mSockfdClient, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));

    // bind ports
    if (bind(mSockfdClient, (struct sockaddr*)&addrClient, addr_len) == -1){
        printf("Client: Failed to bind socket on port %d\n", mPortOut);
        close(mSockfdClient);
        return false;
    }
    // set buffer
    memset(mbuffer, 0, mBuffersize);

    printf("Listen to the port %d\n", mPortOut);
    //puts("Client is listening!");
    return true;
}

void udpSocket::SetViewer(ORB_SLAM2::Viewer *pViewer){
    mpViewer = pViewer;
}

int udpSocket::GenerateRotCmd(cv::Vec3f HIP_C, double thres){
    int DoubleCmd = 0;

    double alpha = atan(HIP_C[0]/HIP_C[2])*180.0/3.1415;

    if (alpha > thres)
        DoubleCmd = 3;
    else if (alpha < -1.0*thres)
        DoubleCmd = 4;
    else
        DoubleCmd = 0;

    return DoubleCmd;
}

int udpSocket::GenerateForwardControlCmd(cv::Vec3f HIP_C, double angleThres, double distThresMin, double distThresMax){
    int DoubleCmd = 0;

    double dist, alpha;
    dist = HIP_C[2];
    if (dist == 0){alpha = 0;}
    else {alpha =atan(HIP_C[0]/HIP_C[2])*180.0/3.1415;}

    //cout << alpha << " " << abs(alpha) << " " << dist << " " << angleThres << " " << distThresMax << " " << distThresMin << endl;

    if (abs(alpha) < angleThres && dist > distThresMin && dist < distThresMax)
        DoubleCmd = 0; // drive = turn = 0.0;
    else if (abs(alpha) < angleThres && dist < distThresMin )
        DoubleCmd = 2; // drive = -0.5; turn = 0.0;
    else if (abs(alpha) < angleThres && dist > distThresMax )
        DoubleCmd = 1; // drive = 0.5; turn = 0.0;
    else if (alpha > angleThres && dist > distThresMin && dist < distThresMax)
        DoubleCmd = 3; // drive = 0.0; turn = 0.5;
    else if (alpha < -1.0*angleThres && dist > distThresMin && dist < distThresMax)
        DoubleCmd = 4; // drive = 0.0; turn = -0.5;
    else if (alpha > angleThres && dist < distThresMin )
        DoubleCmd = 7; // drive -0.5; turn = 0.5;
    else if (alpha > angleThres && dist > distThresMax)
        DoubleCmd = 5; // drive = turn = 0.5;
    else if (alpha < -1.0*angleThres && dist < distThresMin )
        DoubleCmd = 8; // drive = turn = -0.5;
    else if (alpha < -1.0*angleThres && dist > distThresMax)
        DoubleCmd = 6; // drive = 0.5; turn = -0.5;
    else
        DoubleCmd = 0;

    return DoubleCmd;

}

int udpSocket::GenerateBackwardControlCmd(cv::Vec3f HIP_C, double angleThres, double distThresMin, double distThresMax){
    int DoubleCmd = 0;

    return DoubleCmd;
}

}
