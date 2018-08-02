//
// Created by skaegy on 31/07/18.
//

#include "ArucoDetect.h"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

ArucoDetector::ArucoDetector(const string strArucoSettingFile, const string strArucoParamsFile) {
    camMatrix = Mat::eye(3, 3, CV_32F);
    distCoeffs = Mat::zeros(5, 1, CV_32F);

    bool readCPOk = ArucoDetector::readCameraParameters(strArucoSettingFile, camMatrix, distCoeffs);
    if (!readCPOk) {
        cerr << "Invalid camera parameters (ORB setting) file" << endl;
    }
    cout << camMatrix << endl;
    cout << distCoeffs << endl;

    detectorParams = aruco::DetectorParameters::create();
    bool readAPFOk = ArucoDetector::readArucoParameters(strArucoParamsFile, detectorParams);
    if (!readAPFOk) {
        cerr << "Invalid Aruco parameters file" << endl;
    }

    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
}

void ArucoDetector::Run() {

    while (1) {
        if (mlArucoLoadImage.size() != 0) {
            //cout << int(mlArucoLoadImage.size()) << endl;
            Mat imAruco = mlArucoLoadImage.back();
            mlArucoLoadImage.pop_back();
            vector<int> ids;
            vector<vector<Point2f> > corners, rejected;
            vector<Vec3d> rvecs, tvecs;
            unique_lock<mutex> lock(mMutexImage);
            aruco::detectMarkers(imAruco, dictionary, corners, ids, detectorParams, rejected);

            if (ids.size() > 0) {
                aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
                aruco::drawDetectedMarkers(imAruco, corners, ids);
                if (estimatePose == 1) {
                    for (unsigned int i = 0; i < ids.size(); i++)
                        aruco::drawAxis(imAruco, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
                mlArucoShowImage.push_front(imAruco);
            }
        }
    }
}

bool ArucoDetector::isStopped() {
    unique_lock<mutex> lock(mMutexImage);
    mStopped = true;
    return mStopped;
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
    fs["Aruco.estimatePose"] >> estimatePose;
    fs["Aruco.markerLength"] >> markerLength;

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
    Mat imForAruco;
    im.copyTo(imForAruco);
    mlArucoLoadImage.push_front(imForAruco);
}

cv::Mat ArucoDetector::DrawAruco() {
    //unique_lock<mutex> lock(mMutexImage);
    Mat DrawMat;
    if (mlArucoShowImage.size()!=0){
        DrawMat = mlArucoShowImage.back();
        mlArucoShowImage.pop_back();
    }
    //mMutexImage.unlock();
    return DrawMat;
}

} // ORB_SLAM2