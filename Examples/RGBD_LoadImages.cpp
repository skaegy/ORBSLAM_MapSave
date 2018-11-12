//
// Created by root on 08/09/18.
//

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <fstream>
#include <chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesDepth, vector<double> &vTimestamps);

int main()
{

    /// ======================= Load setting files ========================== ///
    const string &strSettingPath = "/home/skaegy/Projects/Cplus_Project/ORB_Tracking/Examples/Setting.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string videoSoure = fSettings["Video_source"];
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];
    const string strImagePath = fSettings["LoadImagePath"];
    fSettings.release();

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;
    bool bHumanPose = false;
    if (1 == HumanPose)
        bHumanPose = true;
    bool bArucoDetect = false;
    if (1 == ArucoDetect)
        bArucoDetect = true;

    /// ============ Retrieve paths to images =============== ///
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = strImagePath + "/association.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc, strCamSet, strArucoParamsFile, strOpenposeSettingFile,
                           ORB_SLAM2::System::RGBD, true, bReuseMap, bHumanPose, bArucoDetect, strMapPath);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imAruco, imOP;
    bool OpStandBy = false, ARUCOStandBy = false;

    while(bHumanPose){
        OpStandBy = SLAM.mpOpDetector->OpStandBy;
        if (OpStandBy){
            break;
        }
    }

    for(int ni=0; ni<nImages; ++ni)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(strImagePath + "/" + vstrImageFilenamesRGB[ni],cv::IMREAD_ANYCOLOR);
        imD = cv::imread(strImagePath + "/" + vstrImageFilenamesD[ni],cv::IMREAD_ANYDEPTH);

        imD.setTo(cv::Scalar(0), imD > 6000);
        cv::Mat imD_medSmooth;
        cv::medianBlur ( imD, imD_medSmooth, 5);
        cv::Mat imD_bilaSmooth, imD_output;
        imD_medSmooth.convertTo(imD_bilaSmooth, CV_32FC1);
        cv::bilateralFilter(imD_bilaSmooth, imD_output,7, 4.3e7, 2);
        imD_output.convertTo(imD_output, CV_16UC1);

        cout << "Frame: " << ni+1 ;
        double tframe = vTimestamps[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        // Pass the image to the ARUCO marker detection system
        imRGB.copyTo(imAruco);
        if (ARUCOStandBy)
            SLAM.mpArucoDetector->ArucoLoadImage(imAruco, tframe);


        // Pass the image to Openpose system
        imRGB.copyTo(imOP);
        if (OpStandBy)
            SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, tframe);

        // Wait for openpose
        int SLAMFrame = ni + 1;
        int OpFrame;

        while (bHumanPose){
            OpFrame = SLAM.mpOpDetector->mFramecnt;
            if (OpFrame>=SLAMFrame){
                break;
            }
        }


        //cout << "Processed frame: " << SLAMFrame << " " << OpFrame <<endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << " Timecost: " << ttrack << endl;
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e3);

    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save map points and trajectory

    SLAM.SaveMap("Map_RGBD_LoadImage.bin");
    SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.txt");
    SLAM.SaveCameraTrajectory("CameraTrajectory.txt");
    if (bHumanPose){
        SLAM.SaveSkeletonTimeStamp("SkeletonTimeStamp.txt");
        SLAM.SaveSkeletonTrajectory("SkeletonTrajectory.txt");
    }


    sleep(5);

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesDepth, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}