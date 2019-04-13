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
    const string &strSettingPath = "../Setting.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string videoSoure = fSettings["Video_source"];
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int UseViewer = fSettings["is_UseViewer"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    const string strArucoParamsFile = fSettings["Aruco_Parameters"];
    int ArucoDetect = fSettings["is_DetectMarker"];
    const string strOpenposeSettingFile = fSettings["Openpose_Parameters"];
    int HumanPose = fSettings["is_DetectHuman"];
    const string strImagePath = fSettings["LoadImagePath"];
    string SampleName;
    for (int i = 0; i < strImagePath.length(); i++)
    {
        if (strImagePath.at(strImagePath.length()-i-1) == '/'){
            SampleName = strImagePath.substr(strImagePath.length()-i,strImagePath.length());
            break;
        }
    }
    printf("----------------------------------\n");
    cout << "Output: " << SampleName <<endl;

    fSettings.release();

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;
    bool bUseViewer = false;
    if (1 == UseViewer)
        bUseViewer = true;
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
                           ORB_SLAM2::System::RGBD, bUseViewer, bReuseMap, bHumanPose, bArucoDetect, strMapPath);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imAruco, imOP;
    bool OpStandBy = false;

    while(bHumanPose && !OpStandBy){
        OpStandBy = SLAM.mpOpDetector->OpStandBy;
        if (OpStandBy){
            break;
        }
        usleep(1000);
    }

    int OpFrame;
    int startFrame = 0;
    for(int ni=startFrame; ni<nImages; ++ni)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(strImagePath + "/" + vstrImageFilenamesRGB[ni],cv::IMREAD_ANYCOLOR);
        imD = cv::imread(strImagePath + "/" + vstrImageFilenamesD[ni],cv::IMREAD_ANYDEPTH);

        /*
         * Depth filter
        imD.setTo(cv::Scalar(0), imD > 5000);
        cv::Mat imD_medSmooth;
        cv::medianBlur ( imD, imD_medSmooth, 5);
        cv::Mat imD_bilaSmooth, imD_output;
        imD_medSmooth.convertTo(imD_bilaSmooth, CV_32FC1);
        cv::bilateralFilter(imD_bilaSmooth, imD_output,7, 4.3e7, 2);
        imD_output.convertTo(imD_output, CV_16UC1);
         */

        cout << "Frame: " << ni+1 << endl;
        double tframe = vTimestamps[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to Openpose system
        imRGB.copyTo(imOP);
        if (OpStandBy){
            /*
             * Use Mask as the input to OPENPOSE
            if (!SLAM.mpOpDetector->mlHumanMask.empty()){
                cv::Scalar maskValue = cv::Scalar(0, 0, 0);

                int XMIN = SLAM.mpOpDetector->mXMIN;
                int XMAX = SLAM.mpOpDetector->mXMAX;
                int YMIN = SLAM.mpOpDetector->mYMIN;
                int YMAX = SLAM.mpOpDetector->mYMAX;
                //cout << XMIN << " " << XMAX << " " << YMIN << " " << YMAX << endl;
                imOP.colRange(0,XMIN).setTo(maskValue);
                imOP.colRange(XMAX,imOP.cols-1).setTo(maskValue);
                imOP.rowRange(0,YMIN).setTo(maskValue);
                imOP.rowRange(YMAX,imOP.rows-1).setTo(maskValue);
            }
            */

            SLAM.mpOpDetector->OpLoadImageRGBD(imOP, imD, tframe);
            }

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        // Wait for ALL FRAME ARE PROCESSED
        int SLAMFrame = ni + 1;

        while (bHumanPose){
            usleep(1000);
            OpFrame = SLAM.mpOpDetector->mFramecnt + startFrame + 1;
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
    if (!bReuseMap)
        SLAM.SaveMap("Map_RGBD_LoadImage.bin");

    SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.txt");
    SLAM.SaveCameraTrajectory("CameraTrajectory.txt");
    if (bHumanPose){
        SLAM.SaveSkeletonTimeStamp(SampleName + "_SkeletonTimeStamp"+".txt");
        SLAM.SaveSkeletonTrajectory(SampleName+ "_SkeletonTrajectory"+".txt");
        SLAM.SaveSkeletonRawTrajectory(SampleName+"_RawSkelTrajectory"+".txt");
        SLAM.SaveOpenposeTrajectory(SampleName+"_OpTrajectory"+".txt");
    }
    if (bReuseMap){
        SLAM.SaveCameraLocTrajectory(SampleName+"_CamLocTrajectory"+".txt");
    }

    //SLAM.Shutdown();

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
            vstrImageFilenamesDepth.push_back(sD);

        }
    }
}