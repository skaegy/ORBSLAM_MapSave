/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * System.cc
 * ORB-SLAM: borrow the idea from PTAM
 * 1) Rubble --> ORB feature
 * 2) DBoW2 --> Place recognition --> Loop closing
 * 3) Strasdat --> Loop update
 * 4) Covisibility graph
 * 5) Kuemmerle & Grisetti --> g2o (graph optimization)
 *
 * Input:
 * 1) Input image:
 *      - Monocular --> GrabImageMonocular(im)
 *      - Stereo --> GrabImageStereo(imLeft, imRight)
 *      - RGB-D --> GrabImageRGBD(imRGB, imD)
 *
 * 2) Convert to grayscale image
 *      - Monocular --> mImGray
 *      - Stereo --> mImGray, mImGrayRight
 *      - RGB-D --> mImGray, imDepth
 * 3) Build Frame
 * [ORBextractor* mpIniORBextractor, *mpORBextractorLeft, mpORBextractorRight]
 *      - Monocular (NOT_INITIALIZED) --> Frame(mImGray, mpIniORBextractor)
 *      - Monocular (INITIALIZED) --> Frame(mImGray, mpORBextractorLeft)
 *      - Stereo --> Frame(mImGray, mImGrayRight, mpORBextractorLeft, mpORBextractorRight)
 *      - RGB-D --> Frame(mImGray, imDepth, mpORBextractorLeft)
 * 4) Track()
 * Then go to the tracking thread --> Tracking.cc

 ** Three main threads **
 *  [1] Tracking
 *      - Extract ORB features
 *      - Estimate pose
 *      - Pose refinement
 *      - Key frame selection
 *  [2] Mapping
 *      - Insert Key Frame --> Update Maps
 *      - Remove outliers, validate the map points
 *      - Triangular --> Generate new map points
 *      - Local buddle adjustment --> Key frame and adjacent key frames
 *      - Validate Key Frame --> Remove repeat key frames
 *  [3] Loop Closing
 *      - Select similar frame --> DBOW2
 *      - Closed Loop Detection (3D <-> 3D transformation, scale! RANSAC)
 *      - Update map
 *      - Graph Optimization --> g2o, update all the map points
 *
 **/


#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

namespace ORB_SLAM2
{

// Initialization --> Vocabulary, setting file, sensor
System::System(const string &strVocFile, const string &strSettingsFile,
        const string &strArucoParamsFile, const string &strOpenposeSettingsFile,
        const eSensor sensor, const bool bUseViewer, const bool bReuse, const bool bHumanPose, const bool bARUCODetect,
        const string &mapFilePath ):
        mSensor(sensor),mbReset(false),
        mbActivateLocalizationMode(bReuse),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions." << endl << endl;

    cout << "OPENPOSE Copyright (C): Multi-person keypoint detection" << endl <<
            " ---- Software licence agreement ----" << endl <<
            " Academic or non-profit organization noncommercial research use only." << endl <<
            " By using or downlowding the software, you are agree to the terms of this licence agreement." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    // 1. Load ORB Vocabulary
    /* Use [new] to create object,
     * Similar to apply for RAM, return the pointer
     * Need to be [delete] after usage*/
    mpVocabulary = new ORBVocabulary();
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // 2. Create KeyFrame Database
    // Create [mpKeyFrameDatabase-->pointer] according to the feature dictionary [mpVocabulary]
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // 3. Create the Map
    // New map or load map
    if (!bReuse){
        mpMap = new Map();
    }
	else{
        LoadMap(mapFilePath.c_str());
        //mpKeyFrameDatabase->set_vocab(mpVocabulary);

        vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it)
        {
            (*it)->SetKeyFrameDatabase(mpKeyFrameDatabase);
            (*it)->SetORBvocabulary(mpVocabulary);
            (*it)->SetMap(mpMap);
            (*it)->ComputeBoW();
            mpKeyFrameDatabase->add(*it);
            //TODO: Time cost is high
            (*it)->SetMapPoints(mpMap->GetAllMapPoints());
            (*it)->SetSpanningTree(vpKFs);
            (*it)->SetGridParams(vpKFs);
            // Reconstruct map points Observation
        }
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << " Timecost: " << ttrack << endl;

        t1 = std::chrono::steady_clock::now();
        vector<ORB_SLAM2::MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for (vector<ORB_SLAM2::MapPoint*>::iterator mit = vpMPs.begin(); mit != vpMPs.end(); ++mit)
        {
            (*mit)->SetMap(mpMap);
            (*mit)->SetObservations(vpKFs);
        }
        t2 = std::chrono::steady_clock::now();
        ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << " Timecost: " << ttrack << endl;

        t1 = std::chrono::steady_clock::now();
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it)
        {
            (*it)->UpdateConnections();
        }
        t2 = std::chrono::steady_clock::now();
        ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << " Timecost: " << ttrack << endl;
	}
	cout << endl << mpMap <<" : is the created map address" << endl;


    // 4. Create Frame Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuse); // Show KeyFrames
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile); // Show Map Points

    // 5. Initialize the Tracking thread, not launch
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuse);

    // 6. Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    // 7. Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    // 0. Initialize the openpose detector thread and launch
    mbHumanPose = bHumanPose;
    mpOpDetector = new OpDetector(strOpenposeSettingsFile, bHumanPose, mSensor);
    if (bHumanPose)
        mptOpDetector = new thread(&ORB_SLAM2::OpDetector::Run, mpOpDetector);


    // 8. Initialize the ARUCO detector thread and launch
    mpArucoDetector = new ArucoDetector(strSettingsFile, strArucoParamsFile);
    if (bARUCODetect)
        mptArucoDetector = new thread(&ORB_SLAM2::ArucoDetector::Run, mpArucoDetector);

    // 9. Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker, mpArucoDetector, mpOpDetector,
            strSettingsFile, bReuse, bHumanPose, bARUCODetect);
    if(bUseViewer)
        mptViewer = new thread(&Viewer::Run, mpViewer);

    // 9-- Initialize the UDP thread and launch
    mpUDPsocket = new udpSocket(strSettingsFile);
    if (bHumanPose)
        mptUDPsocket = new thread(&ORB_SLAM2::udpSocket::RunServer, mpUDPsocket);

    // 10. Set pointers between threads
    // Link threads (Tracker --> Local mapper & Loop Closer & Tracker)
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpTracker->SetViewer(mpViewer);
    mpTracker->SetOpDetector(mpOpDetector);

    // Link threads (Local Mapper --> Tracker & Loop Closer)
    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    // Link threads (LoopCloser --> Tracker & Local mapper)
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // Link threads (ArucoDetector --> Viewer)
    mpArucoDetector->SetViewer(mpViewer);
    mpOpDetector->SetViewer(mpViewer);
    mpUDPsocket->SetViewer(mpViewer);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    /* TrackStereo:
     * Input:
     *      cv::Mat &imLeft --> Left image
     *      cv::Mat &imRight --> Right image
     *      const double &timestamp --> timestamp
     * Return:
     *      mpTracker -> GrabImageStereo(imLeft,imRight,timestamp);
     *      cv::Mat --> Camera pose
     */
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change:
    // 1) Tracking + Localization
    // 2) Tracking + Localization + Mapping
    {
        /*
		* class 'unique_lock' 是一个一般性质的 mutex 属主的封装，
		* 提供延迟锁定（deferred locking），
		* 限时尝试（time-constrained attempts），
		* 递归锁定（recursive locking），
		* 锁主的转换，
		* 以及对条件变量的使用。
		*/
        unique_lock<mutex> lock(mMutexMode);
        // Localization mode: Tracking + Localization --> Close mapping
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            // Launch Tracker thread
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        // Mapping mode: Tracking + Localization + Mapping
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release(); // Release local mapper thread
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    /* TrackStereo:
     * Input:
     *      cv::Mat &im --> RGB image
     *      cv::Mat &depthmap --> Depth image
     *      const double &timestamp --> timestamp
     * Return:
     *      mpTracker->GrabImageRGBD(im,depthmap,timestamp);
     *      cv::Mat --> Camera pose
     */
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change:
    // 1) Tracking + Localization
    // 2) Tracking + Localization + Mapping
    {
        unique_lock<mutex> lock(mMutexMode);
        // Tracking + Localization
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        // Tracking + Mapping
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
    /* 1) Initialization (t=1)
     * Feature (current frame) > 500 --> Initialization
     * Frame 1 = Key frame, pose = [I, 0]
     * Calculate 3D points according to the parallax
     * Generate map points --> & features & update orientation and distance of map
     *                         & map points of key frame & add points on map
     * Show map --> Transformation from reference frame to current frame
     *              Tcr = mTcw * mTwr
     *
     * 2) t>1
     *      a) Motion exists --> Track the last frame
     *         ---- Track failure --> Track reference key frame
     *      b) No motion --> Track the last key frame (key reference frame)
     *         ---- Track local map
     *         ---- Transformation from reference frame to current frame: Tcr = mTcw * mTwr
     */
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    cv::Mat Tcw;
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change:
    // 1) Tracking + Localization
    // 2) Tracking + Localization + Mapping
    {
        unique_lock<mutex> lock(mMutexMode);
        // Tracking + Localization
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        // Tracking + Localization + Mapping
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    //return (mpTracker->GrabImageMonocular(im,timestamp)).clone();
    return (mpTracker->GrabImageMonocular(im,timestamp));
    /* 1) Initialization (t=1)
     * Condition --> No. of features (two adjacent frames) > 100 && No. of matching features > 100
     * Frame 1 = Key frame, pose = [I, 0]
     * Frame 2 --> Global optimization to [R t] & 3D points
     * Generate map points by minimizing the reprojecting error of BA --> Map optimization & Pose optimization & 3D map points
     * Normalization the pose/position of Frame 2 --> Translational vector and coordinates of the map points
     * Show update
     * 2) t> 2
     */

}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::ActivateArucoDetectionMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateArucoDetectionMode = true;
}

void System::DeactivateArucoDetectionMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateArucoDetectionMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::SaveMapRequest()
{
    SaveMap("Slam_latest_Map.bin");
}

void System::ShutdownRequest()
{
    mbShutdown = true;
}

bool System::isShutdown()
{
    return mbShutdown;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    //mpViewer->RequestFinish();
    //mpArucoDetector->RequestFinish();

    // Wait until all thread have effectively stopped
    /*
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() )
    {
        usleep(5000);
    }
     */

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::LoadMap(const string &filename){

    std::ifstream is(filename);
    {
        boost::archive::binary_iarchive ia(is, boost::archive::no_header);
        //ia >> mpKeyFrameDatabase;
        ia >> mpMap;

    }
    is.close();
    cout << endl << filename <<" : Map Loaded!" << endl;
}

void System::SaveMap(const string &filename){
    std::ofstream os(filename);
    {
        ::boost::archive::binary_oarchive oa(os, ::boost::archive::no_header);
        //oa << mpKeyFrameDatabase;
        oa << mpMap;
    }
    os.close();
    cout << endl << "Map saved to " << filename << endl;
}

void System::SaveSkeletonRequest(){
    SaveSkeletonTrajectory("SkeletonTrajectory.txt");
    SaveSkeletonTimeStamp("SkeletonTimestamp.txt");
}

void System::SaveSkeletonTimeStamp(const string &filename){
    vector<double> timestamp = mpOpDetector->mvTimestamp;
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    if (timestamp.size()>0){
        for(vector<double>::iterator lit=timestamp.begin(),lend=timestamp.end();lit!=lend;lit++)
        {
            double ts = *lit;

            f << setprecision(6) << ts/1e3 << endl;
        }
    }

    f.close();

}

void System::SaveSkeletonTrajectory(const string &filename){
    cout << endl << "Saving human skeleton trajectory to " << filename << " ..." << endl;
    vector<cv::Mat> Joints3D = mpOpDetector->mvJoints3DEKF;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    if (Joints3D.size()>0){
        for(vector<cv::Mat>::iterator lit=Joints3D.begin(),lend=Joints3D.end();lit!=lend;lit++)
        {
            cv::Mat Joints = *lit;

            f << setprecision(3) << Joints << endl;
        }
    }

    f.close();

    cout << endl << "Skeleton trajectory saved!" << endl;

}

void System::SaveTrajectoryRequest(){
    SaveKeyFrameTrajectory("KeyFrameTrajectory.txt");
    SaveCameraTrajectory("CameraTrajectory.txt");
}

void System::SaveCameraTrajectory(const string &filename){
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT/1e3 << " " <<  setprecision(6) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectory(const string &filename){
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();

        f << setprecision(6) << (pKF->mTimeStamp)/1e3 << setprecision(6) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveStereoKeyFrameTrajectory(const string &filename){
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

} //namespace ORB_SLAM
