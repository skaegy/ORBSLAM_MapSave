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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "ArucoDetect.h"
#include "DetectHumanPose.h"


#include <mutex>
#include <limits>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class ArucoDetector;
class OpDetector;

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking,
            ArucoDetector* pArucoDetector, OpDetector* pDetector,
            const string &strSettingPath, const bool bReuse, const bool bHumanPose, const bool bARUCODetect);
    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

	struct HumanJointAngles{
		double LThigh, RThigh, LShank, RShank;
		double LAnkle, RAnkle, LKnee, RKnee;
		double LFoot, RFoot, LFTP, RFTP;
	}mJointAngles;

	enum{
		HEAD = 0, SHOULDER_C = 1, SHOULDER_R = 2, SHOULDER_L = 5,
		ELBOW_R = 3, HAND_R = 4, ELBOW_L = 6, HAND_L = 7,
		HIP_C = 8, HIP_R = 9, HIP_L = 12,
		KNEE_R = 10, ANKLE_R = 11, TOE_IN_R = 22, TOE_OUT_R = 23, HEEL_R = 24,
		KNEE_L = 13, ANKLE_L = 14, TOE_IN_L = 19, TOE_OUT_L = 20, HEEL_L = 21
	};

	cv::Vec3f mHIP_C;

	cv::Mat mHumanMask;

private:
    void Draw3DJoints(cv::Mat Joints3D);

	void Draw3DLowerJoints(cv::Mat Joints3D, int mode);

    void Draw2DHumanLoc(cv::Mat Joints3D);

    void Draw2DCamLoc(pangolin::OpenGlMatrix &Twc);

    double AngleLink2Plane(cv::Vec3f point3d, cv::Mat plane3d);

    double AnglePlane2Plane(cv::Mat plane1, cv::Mat plane2);

    double AngleLink2Link(cv::Vec3f point1, cv::Vec3f point_mid, cv::Vec3f point2);

    cv::Mat DrawSkel2DView(cv::Mat Joints3D, cv::Size ImgSize, bool FrontViewFlag);

    void Draw3Dtrj(std::vector<cv::Mat> Joints3D, int N_history);

	void Draw2Dtrj(std::vector<cv::Mat> Joints3D, cv::Mat& Img, bool FrontViewFlag, int N_history);

    cv::Mat CalcHumanBodyCoord(cv::Vec3f HIP_R, cv::Vec3f HIP_C, cv::Vec3f HIP_L);

    void CalcHumanJointAngles(cv::Mat Joints3D, struct HumanJointAngles *mJointAngles, pangolin::OpenGlMatrix &Twc);

    double CalcLinkLength(cv::Vec3f point1, cv::Vec3f point2);

    bool Stop();

	bool mbReuse;
	bool mbHumanPose;
	bool mbARUCODetect;
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    ArucoDetector* mpArucoDetector;
    OpDetector* mpOpDetector;

    // 1/fps in ms
    double mT;
    int mSensor, mTrjHistory;
    float mImageWidth, mImageHeight;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mWindowSizeX, mWindowSizeY;
    float mCamZ, mCameraSize;

	pangolin::OpenGlMatrix Twc;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

