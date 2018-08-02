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
#include<opencv2/core/core.hpp>
#include"System.h"
#include"ArucoDetect.h"

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
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    const string strArucoSettingsFile = strCamSet;

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;

    // Create ArucoDetector system.
    //ORB_SLAM2::ArucoDetector ArucoDetector(strArucoSettingsFile, strArucoParamsFile);
    //std::thread* mptArucoDetector;
    //mptArucoDetector = new thread(&ORB_SLAM2::ArucoDetector::Run, &ArucoDetector);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc,strCamSet,strArucoParamsFile, ORB_SLAM2::System::MONOCULAR,true, bReuseMap,strMapPath);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imSlam, imAruco;
    cv::VideoCapture capture("/dev/video2");
while(1)
    {
        // Read image from file
        capture >> imSlam;
        imSlam.copyTo(imAruco);
        if(imSlam.empty())
        {
            cerr << endl << "Failed to load image!" << endl;
            return 1;
        }
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imSlam, 0);
        SLAM.mpArucoDetector->ArucoLoadImage(imAruco, 0);

        //if(SLAM.isShutdown())
            //break;
    }
    // Stop all threads
    //SLAM.Shutdown();
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
