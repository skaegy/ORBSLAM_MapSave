//
// Created by skaegy on 31/07/18.
//

#ifndef ARUCODETECT_H
#define ARUCODETECT_H

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

    void ArucoLoadImage(const cv::Mat &im, const double &timestamp);

    void SetViewer(Viewer* pViewer);

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    list<cv::Mat> mlLoadImage;
    struct ArucoDrawer{
        vector< int > ids;
        vector< vector< cv::Point2f > > corners, rejected;
        vector< cv::Vec3d > rvecs, tvecs;
        cv::Mat camMatrix, distCoeffs;
        float markerLength;
        int estimatePose;
    }msArucoDrawer;

private:
    bool readCameraParameters(const string strArucoSettingFile, cv::Mat &camMatrix, cv::Mat &distCoeffs);

    bool readArucoParameters(const string strArucoParamsFile, cv::Ptr<cv::aruco::DetectorParameters> &params);

    bool Stop();
    bool CheckFinish();
    void SetFinish();

    // Aruco setting parameters
    int dictionaryId;
    //int estimatePose;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    bool mbFinishRequested;
    bool mbFinished = false;
    std::mutex mMutexFinish;

    bool mbStopped = false;
    bool mbStopRequested;
    std::mutex mMutexStop;

protected:

    std::mutex mMutexImage;

    Viewer* mpViewer;

};

}
#endif //ARUCODETECT_H
