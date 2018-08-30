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

#include "Viewer.h"
#include "MapPoint.h"
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
        ArucoDetector *pArucoDetector, OpDetector *pOpDetector,
        const string &strSettingPath, const bool bReuse, const bool bHumanPose, const bool bARUCODetect):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mpArucoDetector(pArucoDetector), mpOpDetector(pOpDetector),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mbReuse = bReuse;
    mbHumanPose = bHumanPose;
    mbARUCODetect = bARUCODetect;
}

void Viewer::Run()
{
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);

    pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false);
    pangolin::Var<bool> menuSaveCamTrj("menu.Save Cam Trajectory",false,false);
    pangolin::Var<bool> menuSaveSkeleton("menu.Save Skeleton",false,false);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuShutDown("menu.Shut Down",false,false);


    pangolin::Var<double> showPosX("menu.X(red)", 0);
    pangolin::Var<double> showPosY("menu.Y(green)", 0);
    pangolin::Var<double> showPosZ("menu.Z(blue)", 0);
    pangolin::Var<int> NumberOfAruco("menu.ARUCO markers",0, 0, 6);
    pangolin::Var<double> distHipX("menu.HIP_C: X", 0);
    pangolin::Var<double> distHipY("menu.HIP_C: Y", 0);
    pangolin::Var<double> distHipZ("menu.HIP_C: Z", 0);

    pangolin::Var<double> angleLegR("menu.Right leg(rad)", 3.14, 0, 4.0);
    pangolin::Var<double> angleLegL("menu.Left leg(rad)", 3.14, 0, 4.0);
    pangolin::Var<double> angleFootR("menu.Right foot(rad)", 1.57, 0, 2.0);
    pangolin::Var<double> angleFootL("menu.Left foot(rad)", 1.57, 0, 2.0);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // Add Global coordinate system
    pangolin::Renderable tree;
    tree.Add( std::make_shared<pangolin::Axis>() );
    d_cam.SetDrawFunction([&](pangolin::View& view){
        view.Activate(s_cam);
        tree.Render();
    });


    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");
    cv::namedWindow("Openpose");

    bool bFollow = true;
    bool bLocalizationMode = mbReuse;
    mSensor = mpSystem->mSensor;

    const auto time_begin = std::chrono::high_resolution_clock::now();
    double lastTime = 0.0;
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //Draw the Grid every frame
        float xmin=-20.0, xmax=20.0, dx=0.5;
        float zmin=-20.0, zmax=20.0, dz=0.5;
        glBegin(GL_LINES);
        glColor4f(0.1f,0.1f,0.1f,0.3f);
        for(double gridx=xmin; gridx<=xmax; gridx+=dx)
        {
            for(double gridz=zmin; gridz<=zmax; gridz+=dz)
            {
                glVertex3f(gridx, 1.0, zmin);
                glVertex3f(gridx, 1.0, zmax);
                glVertex3f(xmin, 1.0, gridz);
                glVertex3f(xmax, 1.0, gridz);
            }
        }
        glEnd();

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        // Determine to draw different things according to GUI
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        showPosX=Twc.m[12];
        showPosY=Twc.m[13];
        showPosZ=Twc.m[14];

        cv::Mat im = mpFrameDrawer->DrawFrame();

        // Draw detected aruco markers
        if (mbARUCODetect){
            vector<int> ids;
            ids = mpArucoDetector->msArucoDrawer.ids;
            if (ids.size() > 0) {
                NumberOfAruco = ids.size();
                vector<vector<cv::Point2f> > corners, rejected;
                vector<cv::Vec3d > rvecs, tvecs;
                cv::Mat camMatrix, distCoeffs;
                float markerLength;
                int estimatePose;

                corners = mpArucoDetector->msArucoDrawer.corners;
                camMatrix = mpArucoDetector->msArucoDrawer.camMatrix;
                distCoeffs =  mpArucoDetector->msArucoDrawer.distCoeffs;
                markerLength =  mpArucoDetector->msArucoDrawer.markerLength;
                rvecs =  mpArucoDetector->msArucoDrawer.rvecs;
                tvecs =  mpArucoDetector->msArucoDrawer.tvecs;
                estimatePose =  mpArucoDetector->msArucoDrawer.estimatePose;

                // Draw on the 2D image
                cv::aruco::drawDetectedMarkers(im, corners, ids);
                for (unsigned int i = 0; i < ids.size(); i++){
                    if (estimatePose){
                        cv::aruco::drawAxis(im, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                    }
                }

                // Draw point and links the 3D point clound
                glPointSize(15);
                glBegin(GL_POINTS);
                glColor3f(0.2,0.2,0.2);
                for (unsigned int i=0; i<ids.size(); i++){
                    glVertex3f(Twc.m[12] + tvecs[i][0], Twc.m[13] + tvecs[i][1], Twc.m[14] + tvecs[i][2]);
                }
                glEnd();

                glLineWidth(5);
                glColor3f(0.0,0.0,0.8);
                glBegin(GL_LINES);
                for (unsigned int i=0; i<ids.size(); i++){
                    for (unsigned int j=0; j<ids.size(); j++) {
                        if (abs(ids[i]-ids[j])==2){
                            glVertex3f(Twc.m[12] + tvecs[i][0], Twc.m[13] + tvecs[i][1], Twc.m[14] + tvecs[i][2]);
                            glVertex3f(Twc.m[12] + tvecs[j][0], Twc.m[13] + tvecs[j][1], Twc.m[14] + tvecs[j][2]);
                        }
                    }
                }
                glEnd();
            }
        }

        // Draw 2d Human pose
        if(mbHumanPose){
           if (mpOpDetector->mlRenderPoseImage.size()>0){
               cv::Mat OpShow = mpOpDetector->mlRenderPoseImage.front();
               cv::imshow("Openpose", OpShow);
               cv::waitKey(1);
           }
        }

        // Draw 3D human pose (RGB-D)
        if(mbHumanPose && mSensor == 2){
            if (mpOpDetector->mvJoints3DEKF.size()>0){
                cv::Mat Joints3Dekf = mpOpDetector->mvJoints3DEKF.back();

                // Show distance to hip center
                cv::Vec3f hip_c = Joints3Dekf.at<cv::Vec3f>(8);
                if (hip_c[2] > 0){
                    distHipX = Twc.m[12] + hip_c[0];
                    distHipY = Twc.m[13] + hip_c[1];
                    distHipZ = Twc.m[14] + hip_c[2];
                }

                // Show angles
                // Knee R
                if (Joints3Dekf.at<cv::Vec3f>(9)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(10)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(11)[2] > 0) {
                    angleLegR = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(9),
                                                 Joints3Dekf.at<cv::Vec3f>(10),
                                                 Joints3Dekf.at<cv::Vec3f>(11));
                }
                // Knee L
                if (Joints3Dekf.at<cv::Vec3f>(12)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(13)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(14)[2] > 0) {
                    angleLegL = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(12),
                                                 Joints3Dekf.at<cv::Vec3f>(13),
                                                 Joints3Dekf.at<cv::Vec3f>(14));
                }
                // Foot R
                if (Joints3Dekf.at<cv::Vec3f>(10)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(11)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(22)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(23)[2] > 0){

                    cv::Vec3f FootR_mid = (Joints3Dekf.at<cv::Vec3f>(22) + Joints3Dekf.at<cv::Vec3f>(23));
                    angleFootR = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(10),
                                                 Joints3Dekf.at<cv::Vec3f>(11),
                                                 FootR_mid*0.5);
                }

                // Foot L
                if (Joints3Dekf.at<cv::Vec3f>(13)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(14)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(19)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(20)[2] > 0){
                    cv::Vec3f FootL_mid = (Joints3Dekf.at<cv::Vec3f>(19) + Joints3Dekf.at<cv::Vec3f>(20));
                    angleFootL = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(13),
                                                  Joints3Dekf.at<cv::Vec3f>(14),
                                                  FootL_mid*0.5);
                }

                Draw3DJoints(Joints3Dekf);
            }
            /*
            if (mpOpDetector->mvJoints3Draw.size()>0){
                cv::Mat Joints3Draw = mpOpDetector->mvJoints3Draw.back() - 1;
                Draw3DJoints(Joints3Draw);
            }

            if (mpOpDetector->mvJoints3Draw.size()>0 && mpOpDetector->mvJoints3DEKF.size()>0){
                cout << mpOpDetector->mvJoints3Draw.back() - mpOpDetector->mvJoints3DEKF.back() << endl;
            }

            */
        }

        pangolin::FinishFrame();
        const auto now = std::chrono::high_resolution_clock::now();
        const auto totalTimeSec =
                (double) std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_begin).count()
                * 1e-6;
        const auto message = "One frame time: "
                             + std::to_string(totalTimeSec - lastTime) + " milliseconds.";

        //cout << message << endl;

        // Draw ORB-SLAM
        cv::imshow("ORB-SLAM2: Current Frame",im);
        double wait_ms = (mT + lastTime - totalTimeSec);
        if ( wait_ms > 1.0)
            cv::waitKey((int)wait_ms);
        else
            cv::waitKey(1);

        lastTime = totalTimeSec;
        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(menuSaveMap)
        {
            mpSystem->SaveMapRequest();
            menuSaveMap = false;
        }

        if(menuSaveSkeleton)
        {
            mpSystem->SaveSkeletonRequest();
            menuSaveSkeleton = false;
        }

        if(menuSaveCamTrj)
        {
            mpSystem->SaveTrajectoryRequest();
            menuSaveCamTrj = false;
        }

        if(menuShutDown)
        {
            mpSystem->ShutdownRequest();
            menuShutDown = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::Draw3DJoints(cv::Mat Joints3D) {
    // Draw point and links the 3D point clound
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0.2,0.2,0.0);
    vector<cv::Mat> channels(3);
    split(Joints3D, channels);

    for ( int i=0; i < Joints3D.cols; i++){
        // No face
        if (channels[2].at<float>(0,i) > 0 && (i < 15 || i > 18))
            /*
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(i) + Twc.m[1] * channels[1].at<float>(i) + Twc.m[2] * channels[2].at<float>(i),
                       Twc.m[13] + Twc.m[4] * channels[0].at<float>(i) + Twc.m[5] * channels[1].at<float>(i) + Twc.m[6] * channels[2].at<float>(i),
                       Twc.m[14] + Twc.m[8] * channels[0].at<float>(i) + Twc.m[9] * channels[1].at<float>(i) + Twc.m[10] * channels[2].at<float>(i));
                       */
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(i) + Twc.m[4] * channels[1].at<float>(i) + Twc.m[8] * channels[2].at<float>(i),
                       Twc.m[13] + Twc.m[1] * channels[0].at<float>(i) + Twc.m[5] * channels[1].at<float>(i) + Twc.m[9] * channels[2].at<float>(i),
                       Twc.m[14] + Twc.m[2] * channels[0].at<float>(i) + Twc.m[6] * channels[1].at<float>(i) + Twc.m[10] * channels[2].at<float>(i));
    }
    glEnd();


    glLineWidth(5);
    glColor3f(0.0,0.0,0.8);
    int links[2][20] = {{0, 2, 2, 4, 5, 5, 7, 8 ,8 ,8,10,10,22,23,24,13,13,19,20,21},
                        {1, 1, 3, 3, 1, 6, 6, 1, 9,12, 9,11,11,11,11,12,14,14,14,14}};
    glBegin(GL_LINES);
    for(int i=0; i<20; i++){
        int p1 = links[0][i];
        int p2 = links[1][i];
        if (channels[2].at<float>(p1) > 0 && channels[2].at<float>(p2) > 0){
            /*
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(p1) + Twc.m[1] * channels[1].at<float>(p1) + Twc.m[2] * channels[2].at<float>(p1),
                       Twc.m[13] + Twc.m[4] * channels[0].at<float>(p1) + Twc.m[5] * channels[1].at<float>(p1) + Twc.m[6] * channels[2].at<float>(p1),
                       Twc.m[14] + Twc.m[8] * channels[0].at<float>(p1) + Twc.m[9] * channels[1].at<float>(p1) + Twc.m[10] * channels[2].at<float>(p1));
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(p2) + Twc.m[1] * channels[1].at<float>(p2) + Twc.m[2] * channels[2].at<float>(p2),
                       Twc.m[13] + Twc.m[4] * channels[0].at<float>(p2) + Twc.m[5] * channels[1].at<float>(p2) + Twc.m[6] * channels[2].at<float>(p2),
                       Twc.m[14] + Twc.m[8] * channels[0].at<float>(p2) + Twc.m[9] * channels[1].at<float>(p2) + Twc.m[10] * channels[2].at<float>(p2));
                       */
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(p1) + Twc.m[4] * channels[1].at<float>(p1) + Twc.m[8] * channels[2].at<float>(p1),
                       Twc.m[13] + Twc.m[1] * channels[0].at<float>(p1) + Twc.m[5] * channels[1].at<float>(p1) + Twc.m[9] * channels[2].at<float>(p1),
                       Twc.m[14] + Twc.m[2] * channels[0].at<float>(p1) + Twc.m[6] * channels[1].at<float>(p1) + Twc.m[10] * channels[2].at<float>(p1));
            glVertex3f(Twc.m[12] + Twc.m[0] * channels[0].at<float>(p2) + Twc.m[4] * channels[1].at<float>(p2) + Twc.m[8] * channels[2].at<float>(p2),
                       Twc.m[13] + Twc.m[1] * channels[0].at<float>(p2) + Twc.m[5] * channels[1].at<float>(p2) + Twc.m[9] * channels[2].at<float>(p2),
                       Twc.m[14] + Twc.m[2] * channels[0].at<float>(p2) + Twc.m[6] * channels[1].at<float>(p2) + Twc.m[10] * channels[2].at<float>(p2));
        }
    }
    glEnd();

}

double Viewer::AnglePoint2Plane(cv::Vec3f point3d, cv::Mat plane3d){
    double beta = 0.0;

    cv::Vec3f planeVec1 = plane3d.at<cv::Vec3f>(1) - plane3d.at<cv::Vec3f>(0);
    planeVec1 = planeVec1 / (cv::norm(planeVec1) + 1e-23);
    cv::Vec3f planeVec2 = plane3d.at<cv::Vec3f>(2) - plane3d.at<cv::Vec3f>(0);
    planeVec2 = planeVec2 / (cv::norm(planeVec2) + 1e-23);

    if (point3d[2] > 0 && plane3d.at<cv::Vec3f>(0)[2] > 0 && plane3d.at<cv::Vec3f>(1)[2] && plane3d.at<cv::Vec3f>(2)[2]){
        cv::Vec3f planeNormV = planeVec1.cross(planeVec2);
        beta = AnglePoint2Point(point3d, plane3d.at<cv::Vec3f>(0), plane3d.at<cv::Vec3f>(0) + planeNormV);
        beta = beta + 3.1415/2;
    }

    return beta;
}

double Viewer::AnglePoint2Point(cv::Vec3f point1, cv::Vec3f point_mid, cv::Vec3f point2){
    double alpha = 0.0;
    if(point1[2]>0 && point_mid[2]>0 && point2[2]>0){
        cv::Vec3f l1 = point1 - point_mid;
        cv::Vec3f l2 = point2 - point_mid;

        auto innerProduct = l1.t()*l2;
        double num = innerProduct[0];
        double den = cv::norm(l1)*cv::norm(l2);

        if (0 == den){
            alpha = 0.0;
        }
        else{
            alpha = acos(num/den);
        }


    }

    return alpha;
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
