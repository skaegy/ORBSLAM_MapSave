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
    // Index of BODY_25 model
    enum{
        HEAD = 0, SHOULDER_C = 1, SHOULDER_R = 2, SHOULDER_L = 5,
        ELBOW_R = 3, HAND_R = 4, ELBOW_L = 6, HAND_L = 7,
        HIP_C = 8, HIP_R = 9, HIP_L = 12,
        KNEE_R = 10, ANKLE_R = 11, TOE_IN_R = 22, TOE_OUT_R = 23, HEEL_R = 24,
        KNEE_L = 13, ANKLE_L = 14, TOE_IN_L = 19, TOE_OUT_L = 20, HEEL_L = 21
    };

    // Index of BODY_25 model
    struct HumanParams{
        // Link length of different segments
        double Link_thigh_L, Link_thigh_R;
        double Link_hip_L, Link_hip_R;
        double Link_shank_L, Link_shank_R;
        double Link_foot_R, Link_foot_L;
        double Link_heel_L, Link_heel_R;

        // MIN-MAX constraint of different segments (user-defined)
        double link_thigh_MAX, link_thigh_MIN;
        double link_hip_MAX, link_hip_MIN;
        double link_shank_MAX, link_shank_MIN;
        double link_foot_MAX, link_foot_MIN;
        double link_heel_MAX, link_heel_MIN;

        // Joint angles of lower limb
        double Angle_thigh_L, Angle_thigh_R; // Angle between thigh link and the floor plane
        double Angle_shank_L, Angle_shank_R; // Angle between shank link and floor plane
        double Angle_foot_L, Angle_foot_R;   // Angle between foot plane and floor plane
        double Angle_ankle_L, Angle_ankle_R; // Angle between foot plane (link) and shank link
        double Angle_knee_L, Angle_knee_R;   // Angle between thigh link and shank link
        double Angle_ftp_L, Angle_ftp_R;     // Angle between foot direction and foot path (the projection of the HIP_C onto the floor plane)

        // MIN-MAX constraint of joint angles
        double angle_thigh_MAX, angle_thigh_MIN;
        double angle_shank_MAX, angle_shank_MIN;
        double angle_foot_MAX, angle_foot_MIN;
        double angle_ankle_MAX, angle_ankle_MIN;
        double angle_knee_MAX, angle_knee_MIN;
        double angle_ftp_MAX, angle_ftp_MIN;
    };

    // OPENPOSE
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

    void OpLoadImageMonocular(const cv::Mat &im, const double timestamp);

    void OpLoadImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double timestamp);

    void SetViewer(Viewer* pViewer);

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    // List for various images
    list<cv::Mat> mlLoadImage, mlLoadDepth, mlRenderPoseImage, mlHumanMask;
    // List for timestamp
    list<double> mlLoadTimestamp;

    // 2D & 3D joints
    cv::Mat mJoints2D, mJoints2D_last;
    cv::Mat mJoints3D, mJoints3D_last, mJoints3D_EKFsmooth;
    cv::Mat mHumanMask;
    vector<cv::Mat> mvJoints3Draw;
    vector<cv::Mat> mvJoints3DEKF;
    vector<cv::Mat> mvOpJoints2D;
    vector<double> mvTimestamp;
    cv::Vec3i mNormFloor;  // Norm Vector of the floor plane,
    // in this project we first make sure the initial camera pose is parallel to the floor plane,
    // which means that the mNormFloor = [0 -1 0];
    int mXMIN, mXMAX, mYMIN, mYMAX;
    bool OpStandBy = false; // The flag for OP initialization
    int mFramecnt = 0;

private:
    struct KalmanFilterParams{
        double wk;
        double vk;
        double pk;

        int stateNum;
        int measureNum;
    };

    struct CameraParameters{
        double fx, fy;
        double ppx, ppy;
        int width, height;
    };

    cv::Vec3f inline rs_pixel2point(cv::Point2i pixel, double depth);

    cv::Point2i inline rs_point2pixel(cv::Vec3f point);

    cv::Mat Skeleton2Dto3D(cv::Mat& Joints2D, cv::Mat& imD, double renderThres);

    void Skeleton3DSeg(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat& Joints3D, cv::Mat& outputIm);

    void SkeletonSquareMask(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat &outputIm);

    double GetPixelDepth(cv::Point2i pixel, cv::Mat& imD, int depth_radius);

    void Plot2DJoints(cv::Mat Joints2D, cv::Mat& im, double renderThres);

    std::vector<float> unique(const cv::Mat& input, bool sort);

    cv::Mat GetInformPersonJoint(cv::Mat Joints2D, double renderThres, cv::Size Im_size);

    cv::KalmanFilter KFinit(struct KalmanFilterParams *KFparams);

    cv::Mat KFupdate(cv::Mat &Joints3D, const int stateNum, const int measureNum);

    void KFupdate(cv::Mat &Joints3D, cv::Mat &JointsLastFrame, const int stateNum, const int measureNum);

    void InitHumanParams();

    void UpdateHumanParams_LinkLength(cv::Mat &Joints3D, const int window_size);

    cv::Mat SmoothWithLengthConstraint(cv::Mat measurementPt, cv::Vec3f parentPt, float linkConstraint );

    double SkeletonDist(cv::Mat &skel_curr, cv::Mat &skel_last, int *JointSet, const unsigned int SetSize);

    bool Stop();

    bool CheckFinish();

    void SetFinish();

    // Openpose extractor parameters
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    int mOPflag_logging_level, mOPflag_num_gpu, mOPflag_num_gpu_start, mOPflag_scale_number;
    int mOPflag_part_to_show, mOPflag_number_people_max, mOPflag_op_tracking;
    double mOPflag_scale_gap, mOPflag_render_threshold, mOPflag_alpha_pose;
    std::string mOPflag_model_pose, mOPflag_model_folder, mOPflag_net_resolution, mOPflag_output_resolution;

    // Camera Parameters
    CameraParameters mCamParas;

    // KF Parameters
    KalmanFilterParams mKFparameters;
    cv::KalmanFilter KFs3D[25];
    //double mKF_wk, mKF_vk, mKF_pk;

    // Mask Parameters
    float mZ_max_last = 2.0;

    // Sensor
    int mSensor;

    // Human pose & skeleton parameters
    HumanParams mHumanParams; // struct of human body parameters
    bool mbHumanPose = true;
    int mLowerLimbSet[13]={8,9,10,11,12,13,14,19,20,21,22,23,24};
    int mLowerPair[2][12]={{8, 8, 9,10,11,11,11,12,13,14,14,14},
                           {9,12,10,11,22,23,24,13,14,19,20,21}};
    int mFullbodyPair[2][20] = {{0, 2, 2, 4, 5, 5, 7, 8 ,8 ,8,10,10,22,23,24,13,13,19,20,21},
                                {1, 1, 3, 3, 1, 6, 6, 1, 9,12, 9,11,11,11,11,12,14,14,14,14}};

    // Finish flags and mutexs
    bool mbFinishRequested;
    bool mbFinished = false;
    std::mutex mMutexFinish;

    // Stop flags and mutexs
    bool mbStopped = false;
    bool mbStopRequested;
    std::mutex mMutexStop;

protected:
    std::mutex mMutexDepthIm;
    std::mutex mMutexColorIm;
    std::mutex mMutexOutputIm;
    std::mutex mMutexJoint;

    Viewer* mpViewer;
};

}
#endif //DETECTHUMANPOSE_H
