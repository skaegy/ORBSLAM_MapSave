//
// Created by skaegy on 16/08/18.
//
#ifndef DETECTHUMANPOSE_H
#define DETECTHUMANPOSE_H


#include <iostream>
#include <thread>
#include <mutex>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/tracking.hpp>

#include <openpose/headers.hpp>

//#include <openpose/core/headers.hpp>
//#include <openpose/filestream/headers.hpp>
//#include <openpose/gui/headers.hpp>
//#include <openpose/pose/headers.hpp>
//#include <openpose/utilities/headers.hpp>
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

    struct UserDatum : public op::Datum
    {
        bool boolThatUserNeedsForSomeReason;

        UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
                boolThatUserNeedsForSomeReason{boolThatUserNeedsForSomeReason_}
        {}
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

    list<cv::Mat> mlLoadImage, mlLoadDepth, mlRenderPoseImage, mlHumanMask;
    cv::Mat mJoints2D, mJoints3D, mJoints3D_EKFsmooth;
    cv::Mat mHumanMask;
    vector<cv::Mat> mvJoints3Draw;
    vector<cv::Mat> mvJoints3DEKF;
    vector<double> mvTimestamp;

    bool OpStandBy = false;
    int mFramecnt = 0;

private:
    cv::Mat Joints2Dto3D(cv::Mat& Joints2D, cv::Mat& imD, double renderThres);

    void Joints2DSeg(cv::Mat& imRGB, cv::Mat& Joints2D, cv::Mat& outputIm);

    void Joints3DSeg(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat& Joints3D, cv::Mat& outputIm);

    float GetPointDepth(cv::Vec2f, cv::Mat& imD, int depth_radius);

    void Plot2DJoints(cv::Mat Joints2D, cv::Mat& im, double renderThres);

    std::vector<float> unique(const cv::Mat& input, bool sort);

    cv::Mat GetInformPersonJoint(cv::Mat Joints2D, double renderThres, cv::Size Im_size);

    cv::KalmanFilter KFInitialization(const int stateNum, const int measureNum, double wk, double vk, double pk);

    cv::Mat KFupdate(cv::Mat Joints3D, const int stateNum, const int measureNum);

    void InitHumanParams(struct HumanParams *mHumanParams);

    void UpdateHumanParams(cv::Mat Joints3D, struct HumanParams *mHumanParams);

    cv::Mat updateMeasurement(cv::Mat measurementPt, cv::Vec3f  rootPt, double linkConstraint );

    float CalcSkelDist(cv::Mat skel_curr, cv::Mat skel_last, int *JointSet, int JointSize);

    bool Stop();

    bool CheckFinish();

    void SetFinish();

    // Openpose extractor parameters
    int mOPflag_logging_level, mOPflag_num_gpu, mOPflag_num_gpu_start, mOPflag_scale_number;
    int mOPflag_part_to_show, mOPflag_number_people_max, mOPflag_op_tracking;
    double mOPflag_scale_gap, mOPflag_render_threshold, mOPflag_alpha_pose;
    std::string mOPflag_model_pose, mOPflag_model_folder, mOPflag_net_resolution, mOPflag_output_resolution;
    // Camera Parameters
    double mCam_fx, mCam_fy, mCam_ppx, mCam_ppy;
    // KF Parameters
    double mKF_wk, mKF_vk, mKF_pk;
    // Mask Parameters
    float mZ_max_last = 2.0;
    // Sensor
    int mSensor;
    list<double> mlLoadTimestamp;
    HumanParams mHumanParams; // struct of human body parameters


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
    std::mutex mMutexDepthIm;
    std::mutex mMutexColorIm;
    std::mutex mMutexOutputIm;
    std::mutex mMutexMask;
    std::mutex mMutexJoint;

    Viewer* mpViewer;
};

}
#endif //DETECTHUMANPOSE_H
