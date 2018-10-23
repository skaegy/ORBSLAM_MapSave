//
// Created by skaegy on 07/08/18.
//
//TODO: NEED TO CHANGE .back(), .begin() ===> iterator it, *it
#include <iostream>
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

mutex mMutexInput;

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

    // --------------------REALSENSE INITIALIZATION -------------------------
    pipeline pipe;
    config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, IN_FRAME); // Enable default depth
    rs_cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IN_FRAME);
    pipeline_profile rs_device = pipe.start(rs_cfg);
    rs2::device selected_device = rs_device.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    /// Open realsense Emitter
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }
    rs2::align align(RS2_STREAM_COLOR);
    colorizer color_map;

    spatial_filter spat_filter;    //
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temporal_filter temp_filter;   //
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.8);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 50);
    disparity_transform depth_to_disparity(true);
    disparity_transform disparity_to_depth(false);

    // Main loop
    cv::Mat imAruco(IMG_WIDTH, IMG_HEIGHT, CV_8UC3);
    cv::Mat imOP(IMG_WIDTH, IMG_HEIGHT, CV_8UC3);
    bool OpStandBy, ARUCOStandBy;
    list<cv::Mat> processed_color;
    list<cv::Mat> processed_depth;
    list<double> lTimestamp;

    std::thread LoadRealsense([&]() {
        while(1){
            usleep(1000);
            // Read image from realsense
            auto rs_d415 = pipe.wait_for_frames();

            auto aligned_frame = align.process(rs_d415);
            auto depth = aligned_frame.get_depth_frame();
            auto color = aligned_frame.get_color_frame();
            std::chrono::milliseconds unix_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >
                    (std::chrono::system_clock::now().time_since_epoch());
            double unix_timestamp_ms = std::chrono::milliseconds(unix_timestamp).count();
            mMutexInput.lock();
            lTimestamp.push_front(unix_timestamp_ms);
            if (lTimestamp.size() > 2)
                lTimestamp.pop_back();
            mMutexInput.unlock();

            // Filter
            depth = depth_to_disparity.process(depth);
            depth = spat_filter.process(depth);
            depth = temp_filter.process(depth);
            depth = disparity_to_depth.process(depth);

            /*
            cv::Mat imD_bilaSmooth, imD_output;
            imD.convertTo(imD_bilaSmooth, CV_32FC1);
            cv::bilateralFilter(imD_bilaSmooth, imD_output,3, 4.3e7, 16);
            imD_output.convertTo(imD_output, CV_16UC1);
             */

            // realsense frame to mat
            cv::Mat rsRGB(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat rsD(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16UC1, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
            rsD.setTo(cv::Scalar(0), rsD < 300);
            rsD.setTo(cv::Scalar(0), rsD > 6000);

            mMutexInput.lock();
            processed_color.push_front(rsRGB.clone());
            processed_depth.push_front(rsD.clone());
            if (processed_color.size() > 2)
                processed_color.pop_back();
            if (processed_depth.size() > 2)
                processed_depth.pop_back();
            mMutexInput.unlock();
        }
    });
    LoadRealsense.detach();

while(1) {

    usleep(1000);
    if (processed_depth.size() > 0 && processed_color.size() > 0) {
        if (bHumanPose)
            OpStandBy = SLAM.mpOpDetector->OpStandBy;
        if (bArucoDetect)
            ARUCOStandBy = SLAM.mpArucoDetector->ArucoStandBy;

        //std::chrono::milliseconds unix_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >
                //(std::chrono::system_clock::now().time_since_epoch());
        //double unix_timestamp_ms = std::chrono::milliseconds(unix_timestamp).count();

        // Read image from realsense
        mMutexInput.lock();
        double unix_timestamp_ms = lTimestamp.front();
        cv::Mat imD = processed_depth.front();
        cv::Mat imRGB = processed_color.front();
        mMutexInput.unlock();

        // Pass the image to ARUCO marker detection system
        if (ARUCOStandBy){
            imAruco = processed_color.front();
            SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, unix_timestamp_ms);
        }

        // Pass the image to Openpose system
        if (OpStandBy){
            imOP = processed_color.front();
            SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, unix_timestamp_ms);
        }

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, unix_timestamp_ms);
    }

    if (SLAM.isShutdown())
        break;
}
    // Stop realsense
    pipe.stop();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectory("RGBDTrajectory.txt");
    SLAM.SaveSkeletonTrajectory("HumanSkeletonTrajectory.txt");

    // Shutdown all threads
    SLAM.Shutdown();

    return 0;
}
