//
// Created by skaegy on 31/07/18.
//

#ifndef ARUCODETECT_H
#define ARUCODETECT_H

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <list>
#include "System.h"

using namespace std;

namespace ORB_SLAM2{
class System;

class ArucoDetector{
public:
    ArucoDetector(const string strArucoSettingFile, const string strArucoParamsFile);

    void Run();

    bool isStopped();

    void ArucoLoadImage(const cv::Mat &im, const double &timestamp);

    void SetViewer(Viewer* pViewer);

    cv::Mat DrawAruco();

    bool mStopped = false;
    list<cv::Mat> mlArucoLoadImage;
    list<cv::Mat> mlArucoShowImage;

private:

    bool readCameraParameters(const string strArucoSettingFile, cv::Mat &camMatrix, cv::Mat &distCoeffs);

    bool readArucoParameters(const string strArucoParamsFile, cv::Ptr<cv::aruco::DetectorParameters> &params);

    // Aruco setting parameters
    int dictionaryId;
    int estimatePose;
    float markerLength;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Mat camMatrix, distCoeffs;


protected:

    std::mutex mMutexImage;

    Viewer* mpViewer;

};

}
#endif //ARUCODETECT_H
