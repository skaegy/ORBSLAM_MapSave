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
    enum{
        HEAD = 0, SHOULDER_C = 1, SHOULDER_R = 2, SHOULDER_L = 5,
        ELBOW_R = 3, HAND_R = 4, ELBOW_L = 6, HAND_L = 7,
        HIP_C = 8, HIP_R = 9, HIP_L = 12,
        KNEE_R = 10, ANKLE_R = 11, TOE_IN_R = 22, TOE_OUT_R = 23, HEEL_R = 24,
        KNEE_L = 13, ANKLE_L = 14, TOE_IN_L = 19, TOE_OUT_L = 20, HEEL_L = 21
    };

    struct HumanParams{
        double Link_thigh_L, Link_thigh_R, Link_hip_L, Link_hip_R;
        double Link_shank_L, Link_shank_R, Link_foot_R, Link_foot_L;
        double Link_heel_L, Link_heel_R;

        int Cnt_thigh_L, Cnt_thigh_R, Cnt_hip_L, Cnt_hip_R;
        int Cnt_shank_L, Cnt_shank_R, Cnt_foot_R, Cnt_foot_L;
        int Cnt_heel_L, Cnt_heel_R;
    };

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
    int mFramecnt = 0;

private:
    cv::Mat Joints2Dto3D(cv::Mat Joints2D, cv::Mat& imD, double renderThres);

    float GetPointDepth(cv::Vec2f, cv::Mat& imD, int depth_radius);

    void Plot2DJoints(cv::Mat Joints2D, cv::Mat& im, double renderThres);

    std::vector<float> unique(const cv::Mat& input, bool sort);

    cv::Mat GetInformPersonJoint(cv::Mat Joints2D, double renderThres, cv::Size Im_size);

    cv::KalmanFilter KFInitialization(const int stateNum, const int measureNum, double wk, double vk, double pk);

    cv::Mat KFupdate(cv::Mat Joints3D, const int stateNum, const int measureNum);

    //TODO
    cv::Mat RemoveSkelFlip(cv::Mat skel_curr, cv::Mat skel_last);

    float CalcSkelDist(cv::Mat skel_curr, cv::Mat skel_last, int *JointSet, int JointSize);

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
    HumanParams mHumanParams; // struct of human body parameters

    void InitHumanParams(struct HumanParams *mHumanParams);
    void UpdateHumanParams(cv::Mat Joints3D, struct HumanParams *mHumanParams);
    cv::Mat updateMeasurement(cv::Mat measurementPt, cv::Vec3f  rootPt, double linkConstraint );

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
