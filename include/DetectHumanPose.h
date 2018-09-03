//
// Created by skaegy on 16/08/18.
//
#ifndef ORB_SLAM2_DETECTHUMANPOSE_H
#define ORB_SLAM2_DETECTHUMANPOSE_H

#include <iostream>
#include <thread>
#include <mutex>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/tracking.hpp>
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>
#include "System.h"


using namespace std;

namespace ORB_SLAM2{
class System;
class Viewer;

class OpDetector{
public:
    OpDetector(const string &strOpenposeSettingsFile, const bool bHumanPose, const int SensorMode);

    void Run();

    void OpLoadImageMonocular(const cv::Mat &im, const double &timestamp);

    void OpLoadImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);

    void SetViewer(Viewer* pViewer);

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    list<cv::Mat> mlLoadImage, mlLoadDepth, mlRenderPoseImage;
    cv::Mat mJoints2D;
    vector<cv::Mat> mvJoints3Draw;
    vector<cv::Mat> mvJoints3DEKF;
    vector<double> mvTimestamp;

    bool OpStandBy = false;

private:
    cv::Mat Joints2Dto3D(cv::Mat Joints2D, cv::Mat& imD, double renderThres);

    float GetPointDepth(cv::Vec2f, cv::Mat& imD, int depth_radius);

    void Plot2DJoints(cv::Mat Joints2D, cv::Mat& im, double renderThres);

    std::vector<float> unique(const cv::Mat& input, bool sort);

    cv::Mat GetInformPersonJoint(cv::Mat Joints2D, double renderThres, cv::Size Im_size);

    cv::KalmanFilter KFInitialization(const int stateNum, const int measureNum, double wk, double vk, double pk);

    //TODO
    cv::Mat JointsFlipDetection(cv::Mat Joints2D, cv::Mat Joints2Dlast);

    bool Stop();

    bool CheckFinish();

    void SetFinish();

    // Openpose extractor parameters
    int logging_level, num_gpu_start, scale_number;
    double scale_gap, render_threshold, alpha_pose;
    string model_pose, model_folder, net_resolution, output_resolution;
    double fx, fy, ppx, ppy;
    double wk, vk, pk;
    int mSensor;
    list<double> mlLoadTimestamp;


    bool mbFinishRequested;
    bool mbFinished = false;
    std::mutex mMutexFinish;

    bool mbStopped = false;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbHumanPose = true;

    bool isAfterFirst[25] = {0};
    cv::KalmanFilter KFs3D[25];

protected:
    std::mutex mMutexOp;

    Viewer* mpViewer;
};

}
#endif //ORB_SLAM2_DETECTHUMANPOSE_H
