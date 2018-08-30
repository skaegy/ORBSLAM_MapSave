//
// Created by root on 30/08/18.
//

//
// Created by skaegy on 07/08/18.
//

#include<iostream>
#include <vector>
#include <list>
#include <thread>
#include <algorithm>
#include <fstream>
#include <omp.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;
using namespace rs2;

int main()
{
    // ==================== Load settings  ==================== //
    const string &strSettingPath = "../Setting.yaml";


    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string videoSource = fSettings["Video_source"];
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];
    fSettings.release();

    // =================== Load camera parameters =================== //
    cv::FileStorage fs(strCamSet, cv::FileStorage::READ);
    const int IMG_WIDTH = fs["Camera.width"];
    const int IMG_HEIGHT = fs["Camera.height"];
    const int IN_FRAME = fs["Camera.fps"];


    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;
    bool bHumanPose = false;
    if (1 == HumanPose)
        bHumanPose = true;
    bool bArucoDetect = false;
    if (1 == ArucoDetect)
        bArucoDetect = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc, strCamSet, strArucoParamsFile, strOpenposeSettingFile,
                           ORB_SLAM2::System::RGBD, true, bReuseMap, bHumanPose, bArucoDetect, strMapPath);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    bool OpStandBy, ARUCOStandBy;
    cv::Mat imAruco, imOP;
    list<cv::Mat> processed_color;
    list<cv::Mat> processed_depth;
    cv::VideoCapture capture(videoSource);

    std::thread LoadRealsense([&]() {
        while(1){


            if (capture.isOpened()){
                // Read image from RTSP streaming
                cv::Mat imCombine, imD_C3;
                std::vector<cv::Mat> channel(3);
                capture >> imCombine;

                /* Convert Depth (CV_8UC3) to (CV_16UC1)*/
                cv::Mat imRGB(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3);
                cv::Mat imD(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16UC1);

                imCombine.rowRange(0,IMG_HEIGHT).copyTo(imRGB);
                imCombine.rowRange(IMG_HEIGHT,IMG_HEIGHT*2).copyTo(imD_C3);

                for (int i = 0; i< IMG_HEIGHT; i++){
                    for (int j = 0; j < IMG_WIDTH; j++){
                        cv::Vec3b mapData = imD_C3.at<cv::Vec3b>(i,j);
                        ushort depth_z16;
                        if (mapData(1) >= mapData(0) )
                            depth_z16 = (ushort)mapData(0)*20 + (ushort)mapData(2);
                        if (mapData(1) < mapData(0))
                            depth_z16 = (ushort)mapData(0)*19 + (ushort)mapData(1);
                        imD.at<ushort>(i,j) = depth_z16;
                    }
                }

                // Store processed RGB & Depth in the std::list
                processed_color.push_front(imRGB);
                processed_depth.push_front(imD);
                if (processed_color.size() > 1)
                    processed_color.pop_back();
                if (processed_depth.size() > 1)
                    processed_depth.pop_back();
            }
            else{
                cerr << "Can not open streaming video. Please check again!" << endl;
                break;
            }
        }
    });
    LoadRealsense.detach();


    while(1) {
        if (processed_depth.size() > 0 && processed_color.size() > 0) {
            if (bHumanPose)
                OpStandBy = SLAM.mpOpDetector->OpStandBy;
            if (bArucoDetect)
                ARUCOStandBy = SLAM.mpArucoDetector->ArucoStandBy;

            std::chrono::milliseconds unix_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >
                    (std::chrono::system_clock::now().time_since_epoch());
            double unix_timestamp_ms = std::chrono::milliseconds(unix_timestamp).count();

            // Read image from realsense
            cv::Mat imD = processed_depth.front();
            cv::Mat imRGB = processed_color.front();

            // Pass the image to the SLAM system

            SLAM.TrackRGBD(imRGB, imD, unix_timestamp_ms);

            //cout << message << endl;
            // Pass the image to ARUCO marker detection system
            imRGB.copyTo(imAruco);
            if (ARUCOStandBy)
                SLAM.mpArucoDetector->ArucoLoadImage(imAruco, unix_timestamp_ms);

            // Pass the image to Openpose system
            imRGB.copyTo(imOP);
            if (OpStandBy)
                SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, unix_timestamp_ms);
        }

        if (SLAM.isShutdown())
            break;
    }
    // Stop all threads
    // SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectory("RGBDTrajectory.txt");

    SLAM.SaveSkeletonTrajectory("HumanSkeletonTrajectory.txt");

    return 0;
}
