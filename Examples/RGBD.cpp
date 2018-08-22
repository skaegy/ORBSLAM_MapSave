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

int main()
{
    const string &strSettingPath = "../Setting.yaml";
    // Load settings
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    //TODO: Video streaming for RGB + Depth
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];
    fSettings.release();

    // Load camera parameters
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

    // realsense config
    pipeline pipe;
    config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, IN_FRAME); // Enable default depth
    rs_cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IN_FRAME);
    pipeline_profile rs_device = pipe.start(rs_cfg);
    rs2::align align(RS2_STREAM_COLOR);
    colorizer color_map;

    // Main loop
    cv::Mat imAruco, imOP;
    bool OpStandBy, ARUCOStandBy;
    list<cv::Mat> Load_RS_rgb;
    list<cv::Mat> Load_RS_depth;

    /*
    std::thread LoadRealsense([&]() {

        while(1){
            // Read image from realsense
            auto rs_d415 = pipe.wait_for_frames();
            auto aligned_frame = align.process(rs_d415);
            auto depth = aligned_frame.get_depth_frame();
            auto color = aligned_frame.get_color_frame();
            // realsense frame to mat
            cv::Mat imRGB(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat imD(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16UC1, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
            Load_RS_rgb.push_front(imRGB);
            Load_RS_depth.push_front(imRGB);
            if (Load_RS_rgb.size()>1 && Load_RS_depth.size()>1){
                Load_RS_rgb.pop_back();
                Load_RS_depth.pop_back();
            }
        }
    });
    LoadRealsense.detach();
     */

    while(1) {

        if (bHumanPose)
            OpStandBy = SLAM.mpOpDetector->OpStandBy;
        if (bArucoDetect)
            ARUCOStandBy = SLAM.mpArucoDetector->ArucoStandBy;

        // Read image from realsense
        const auto timerBegin = std::chrono::high_resolution_clock::now();
        auto rs_d415 = pipe.wait_for_frames();
        auto aligned_frame = align.process(rs_d415);
        auto depth = aligned_frame.get_depth_frame();
        auto color = aligned_frame.get_color_frame();
        const auto now = std::chrono::high_resolution_clock::now();
        const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                                  * 1e-9;
        const auto message = "Total time: "
                             + std::to_string(totalTimeSec ) + " seconds.";
        // realsense frame to mat
        cv::Mat imRGB(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat imD(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16UC1, (void *) depth.get_data(), cv::Mat::AUTO_STEP);

        // Pass the image to the SLAM system

        SLAM.TrackRGBD(imRGB, imD, 0);

        //cout << message << endl;
        // Pass the image to ARUCO marker detection system
        imRGB.copyTo(imAruco);
        if (ARUCOStandBy)
            SLAM.mpArucoDetector->ArucoLoadImage(imAruco, 0);

        // Pass the image to Openpose system
        imRGB.copyTo(imOP);
        if (OpStandBy)
            SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, 0);

        if(SLAM.isShutdown())
            break;
    }

    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
