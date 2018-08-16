//
// Created by skaegy on 31/07/18.
//

#include "ArucoDetect.h"
#include <opencv2/aruco.hpp>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

ArucoDetector::ArucoDetector(const string strArucoSettingFile, const string &strArucoParamsFile) {
    msArucoDrawer.camMatrix = Mat::eye(3, 3, CV_32F);
    msArucoDrawer.distCoeffs = Mat::zeros(5, 1, CV_32F);
    mbStopped = false;

    bool readCPOk = ArucoDetector::readCameraParameters(strArucoSettingFile, msArucoDrawer.camMatrix, msArucoDrawer.distCoeffs);
    if (!readCPOk) {
        cerr << "Invalid camera parameters (ORB setting) file" << endl;
    }

    detectorParams = aruco::DetectorParameters::create();
    bool readAPFOk = ArucoDetector::readArucoParameters(strArucoParamsFile, detectorParams);
    if (!readAPFOk) {
        cerr << "Invalid Aruco parameters file" << endl;
    }

    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
}

void ArucoDetector::Run() {
    while (!mbStopped) {
        if (mlLoadImage.size()>0)
        {
            cv::Mat BufMat = mlLoadImage.back();
            mlLoadImage.pop_back();
            aruco::detectMarkers(BufMat, dictionary, msArucoDrawer.corners, msArucoDrawer.ids, detectorParams, msArucoDrawer.rejected);
            if (msArucoDrawer.estimatePose && msArucoDrawer.ids.size() > 0) {
                aruco::estimatePoseSingleMarkers(msArucoDrawer.corners, msArucoDrawer.markerLength, msArucoDrawer.camMatrix,
                       msArucoDrawer.distCoeffs, msArucoDrawer.rvecs,   msArucoDrawer.tvecs);
            }
        }
    }
}

bool
ArucoDetector::readCameraParameters(const string strArucoSettingFile, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    FileStorage fs(strArucoSettingFile, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["Camera.fx"] >> camMatrix.at<float>(0, 0);
    fs["Camera.fy"] >> camMatrix.at<float>(1, 1);
    fs["Camera.cx"] >> camMatrix.at<float>(0, 2);
    fs["Camera.cy"] >> camMatrix.at<float>(1, 2);
    fs["Camera.k1"] >> distCoeffs.at<float>(0, 0);
    fs["Camera.k2"] >> distCoeffs.at<float>(1, 0);
    fs["Camera.p1"] >> distCoeffs.at<float>(2, 0);
    fs["Camera.p2"] >> distCoeffs.at<float>(3, 0);
    fs["Camera.k3"] >> distCoeffs.at<float>(4, 0);
    fs["Aruco.dictionaryId"] >> dictionaryId;
    fs["Aruco.estimatePose"] >> msArucoDrawer.estimatePose;
    fs["Aruco.markerLength"] >> msArucoDrawer.markerLength;

    fs.release();
    return true;
}

bool ArucoDetector::readArucoParameters(const string strArucoParamsFile, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(strArucoParamsFile, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Unable to open aruco detector parameter file!" << endl;
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs.release();
    return true;
}

void ArucoDetector::SetViewer(ORB_SLAM2::Viewer *pViewer) {
    mpViewer = pViewer;
}

void ArucoDetector::ArucoLoadImage(const cv::Mat &im, const double &timestamp)
{
    cv::Mat BufMat;
    im.copyTo(BufMat);
    mlLoadImage.push_front(BufMat);
}

void ArucoDetector::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ArucoDetector::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ArucoDetector::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ArucoDetector::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void ArucoDetector::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool ArucoDetector::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ArucoDetector::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void ArucoDetector::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

} // ORB_SLAM2