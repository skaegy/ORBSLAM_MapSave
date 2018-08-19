//
// Created by skaegy on 07/08/18.
//

#include<iostream>
#include <vector>
#include <list>
#include <thread>
#include <algorithm>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;
using namespace rs2;
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IN_FRAME 15


int main()
{
    const string &strSettingPath = "../Setting.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];
    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];

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

    // realsense config
    pipeline pipe;
    config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, IN_FRAME); // Enable default depth
    rs_cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IN_FRAME);
    //rs_cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    //rs_cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    pipeline_profile rs_device = pipe.start(rs_cfg);
    rs2::align align(RS2_STREAM_COLOR);
    colorizer color_map;

    // Main loop
    cv::Mat imAruco, imOP;
    bool OpStandBy;

    while(1) {
        if (!OpStandBy)
            OpStandBy = SLAM.mpOpDetector->OpStandBy;

    // Read image from realsense
    auto rs_d415 = pipe.wait_for_frames();
    auto aligned_frame = align.process(rs_d415);
    auto depth = aligned_frame.get_depth_frame();
    auto color = aligned_frame.get_color_frame();
    // realsense frame to mat
    cv::Mat imRGB(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat imD(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16UC1, (void *) depth.get_data(), cv::Mat::AUTO_STEP);

    if (imRGB.empty()) {
        cerr << endl << "Failed to load image!" << endl;
        return 1;
    }

    // Pass the image to the SLAM system
    SLAM.TrackRGBD(imRGB, imD, 0);
    // Pass the image to the ARUCO marker detection system
    imRGB.copyTo(imAruco);
    SLAM.mpArucoDetector->ArucoLoadImage(imAruco, 0);
    // Pass the image to the openpose system
    imRGB.copyTo(imOP);
    SLAM.mpOpDetector->OpLoadImage(imOP, 0);

    if (SLAM.isShutdown())
        break;
    }

    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
