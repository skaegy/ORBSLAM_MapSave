/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include <vector>
#include <list>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "System.h"

using namespace std;

int main()
{
    const string &strSettingPath = "/home/skaegy/Projects/Cplus_Project/ORB_Tracking/Examples/Setting.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string videoSoure = fSettings["Video_source"];
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];
    fSettings.release();

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
                            ORB_SLAM2::System::MONOCULAR, true, bReuseMap, bHumanPose, bArucoDetect, strMapPath);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im, imSlam, imAruco, imOP;
    cv::VideoCapture capture(videoSoure);
    bool OpStandBy, ARUCOStandBy;
    list<cv::Mat> LoadImage;

    std::thread LoadStreamingImage([&]() {
        while (capture.isOpened())
        {
            cv::Mat im;
            capture >> im;
            if(im.empty())
            {
                cerr << endl << "Failed to load image!" << endl;
            }
            LoadImage.push_front(im);
            if (LoadImage.size() > 2){
                LoadImage.pop_back();
            }
        }
    });
    LoadStreamingImage.detach();

while(1){

    if (bHumanPose)
        OpStandBy = SLAM.mpOpDetector->OpStandBy;
    if (bArucoDetect)
        ARUCOStandBy = SLAM.mpArucoDetector->ArucoStandBy;
    cout << "size:" << LoadImage.size() << endl;

    if (LoadImage.size()>0){
        imSlam = LoadImage.front();
        imSlam.copyTo(imAruco);
        imSlam.copyTo(imOP);
        if(imSlam.empty())
        {
            cerr << endl << "Failed to load image!" << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imSlam, 0);

        // Pass the image to ARUCO marker detection system
        if (ARUCOStandBy)
            SLAM.mpArucoDetector->ArucoLoadImage(imAruco, 0);

        // Pass the image to Openpose system
        if (OpStandBy)
            SLAM.mpOpDetector->OpLoadImage(imOP, 0);

        if(SLAM.isShutdown())
            break;
    }
}
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

