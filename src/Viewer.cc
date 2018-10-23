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
    mCamZ = fSettings["Camera.Z"];

    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mTrjHistory = fSettings["Viewer.TrjHistory"];
    mWindowSizeX = fSettings["Viewer.WindowSizeX"];
    mWindowSizeY = fSettings["Viewer.WindowSizeY"];
    mCameraSize = fSettings["Viewer.CameraSize"];

    mbReuse = bReuse;
    mbHumanPose = bHumanPose;
    mbARUCODetect = bARUCODetect;
}

void Viewer::Run(){
    mbFinished = false;

    pangolin::CreateWindowAndBind("                Mobile Gait System -- Hamlyn Centre",mWindowSizeY,mWindowSizeX);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,0.1);
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);

    pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false);
    pangolin::Var<bool> menuSaveCamTrj("menu.Save CamTrj(Map)",false,false);
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

    pangolin::Var<double> Rshank("menu.R_shank(deg)", 90.0, 0.0, 270.0);
    pangolin::Var<double> Lshank("menu.L_shank(deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> Rankle("menu.R_ankle(deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> Lankle("menu.L_ankle(deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> Rftp("menu.R_ftp(deg)",  0.0, -90.0, 90.0);
    pangolin::Var<double> Lftp("menu.L_ftp(deg)",  0.0, -90.0, 90.0);
    pangolin::Var<double> RThigh("menu.R_thigh (deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> LThigh("menu.L_thigh (deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> RKnee("menu.R_knee (deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> LKnee("menu.L_knee (deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> RFoot("menu.R_foot (deg)",  90.0, -0.0, 270.0);
    pangolin::Var<double> LFoot("menu.L_foot (deg)",  90.0, -0.0, 270.0);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(mWindowSizeY,mWindowSizeX,mViewpointF,mViewpointF,960,540,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );
    pangolin::OpenGlRenderState s_cam_fix(
            pangolin::ProjectionMatrix(mWindowSizeY,mWindowSizeX,mViewpointF,mViewpointF,960,540,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, mViewpointX,0,mViewpointZ,mViewpointX,-1.0, mViewpointZ)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam_fix = pangolin::CreateDisplay()
            .SetBounds(0.0, 0.45, 0.1, 0.7, -mWindowSizeY/mWindowSizeX);

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.55, 1.0, 0.1, 0.7, -mWindowSizeY/mWindowSizeX)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_img_orb = pangolin::Display("orb_slam")
            .SetBounds(0.67, 1.0, 0.7, 1.0, mWindowSizeY/mWindowSizeX);
            //.SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_op = pangolin::Display("openpose_skeleton")
            .SetBounds(0.33, 0.66, 0.7,1.0, mWindowSizeY/mWindowSizeX);
            //.SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_frontSkel = pangolin::Display("frontview_skeleton")
            .SetBounds(0, 0.33, 0.7, 0.85)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_sideSkel = pangolin::Display("sideview_skeleton")
            .SetBounds(0, 0.33, 0.85, 1.0)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // Add Global coordinate system
    /*
    pangolin::Renderable tree;
    tree.Add( std::make_shared<pangolin::Axis>() );
    d_cam.SetDrawFunction([&](pangolin::View& view){
        view.Activate(s_cam);
        tree.Render();
    });
    pangolin::Renderable tree_fix;
    tree_fix.Add( std::make_shared<pangolin::Axis>() );
    d_cam_fix.SetDrawFunction([&](pangolin::View& view){
        view.Activate(s_cam_fix);
        tree_fix.Render();
    });
    */
    Twc.SetIdentity();

    /// OPENCV IMSHOW
    //cv::namedWindow("ORB-SLAM2: Current Frame");
    //cv::namedWindow("Openpose");

    bool bFollow = true;
    bool bLocalizationMode = mbReuse;
    mSensor = mpSystem->mSensor;

    const auto time_begin = std::chrono::high_resolution_clock::now();
    double lastTime = 0.0;
    mJointAngles.LAnkle=90.0;    mJointAngles.RAnkle=90.0;
    mJointAngles.LFoot =90.0;    mJointAngles.RFoot =90.0;
    mJointAngles.LFTP  =90.0;    mJointAngles.RFTP  =90.0;
    mJointAngles.LKnee =90.0;    mJointAngles.RKnee =90.0;
    mJointAngles.LShank=90.0;    mJointAngles.RShank=90.0;
    mJointAngles.LThigh=90.0;    mJointAngles.RThigh=90.0;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ///Draw the Grid every frame
        d_cam.Activate(s_cam);
        float xmin=-20.0, xmax=20.0, dx=0.5;
        float zmin=-20.0, zmax=20.0, dz=0.5;
        glBegin(GL_LINES);
        glColor4f(0.1f,0.1f,0.1f,0.3f);
        for(double gridx=xmin; gridx<=xmax; gridx+=dx)
        {
            for(double gridz=zmin; gridz<=zmax; gridz+=dz)
            {
                glVertex3f(gridx, mCamZ, zmin);
                glVertex3f(gridx, mCamZ, zmax);
                glVertex3f(xmin, mCamZ, gridz);
                glVertex3f(xmax, mCamZ, gridz);
            }
        }
        glEnd();

        d_cam_fix.Activate(s_cam_fix);
        glBegin(GL_TRIANGLE_STRIP);
        glColor4f(0.9, 0.9, 0.9, 0.8);

        glTexCoord2f(0.0f, 0.0f); glVertex3f(-20.0, mCamZ, -20.0);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-20.0, mCamZ, 20.0);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(20.0, mCamZ, -20.0);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(20.0, mCamZ, 20.0);
        glEnd();

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        /// Determine to draw different things according to GUI
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

        /// ---------- Display map points ------------ ///
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        d_cam_fix.Activate(s_cam_fix);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        Draw2DCamLoc(Twc);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints2D();

        showPosX=Twc.m[12];
        showPosY=Twc.m[13];
        showPosZ=Twc.m[14];

        ///--------- display ORB-SLAM  ---------//
        /*
        if (mpOpDetector->mlHumanMask.size() > 0){
            mHumanMask = mpOpDetector->mlHumanMask.front();
        }
        else{
            mHumanMask = cv::Mat::ones(mImageHeight, mImageWidth, CV_8UC1);
        }
        cv::Mat im = mHumanMask.clone();
        */
        cv::Mat im = mpFrameDrawer->DrawFrame();
        // display ORB-SLAM in opencv
        //cv::imshow("ORB-SLAM2: Current Frame",im);
        // display the image in opengl
        pangolin::GlTexture imageTextureORB(im.cols,im.rows,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
        imageTextureORB.Upload(im.data,GL_BGR,GL_UNSIGNED_BYTE);
        d_img_orb.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTextureORB.RenderToViewportFlipY();

        ///--------- display ARUCO marker  ---------//
        /*
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
*/

        ///--------- display 3D human pose  ---------//
        if(mbHumanPose){
            if (mpOpDetector->mvJoints3DEKF.size()>0){
                std::vector<cv::Mat>::iterator it = mpOpDetector->mvJoints3DEKF.end();
                std::vector<cv::Mat>::iterator it_raw = mpOpDetector->mvJoints3Draw.end();
                //cv::Mat Joints3Dekf = mpOpDetector->mvJoints3DEKF.back();
                cv::Mat Joints3Dekf = *(it-1);
                cv::Mat Joints3Draw = *(it_raw-1);
                //std::vector<cv::Mat> mvJoints3DEKF = mpOpDetector->mvJoints3DEKF;

                d_cam.Activate(s_cam);
                Draw3DLowerJoints(Joints3Dekf,0); // Smoothed Skeleton
                //Draw3DLowerJoints(Joints3Draw,1); // Raw Skeleton
                //Draw3Dtrj(mvJoints3DEKF, mTrjHistory);

                d_cam_fix.Activate(s_cam_fix);
                Draw2DHumanLoc(Joints3Dekf);

                // Show distance to hip center
                cv::Vec3f hip_c3D = Joints3Dekf.at<cv::Vec3f>(8);

                /// Use these information for robot following
                if (hip_c3D[2] > 0){
                    distHipX = hip_c3D[0]; distHipY = hip_c3D[1]; distHipZ = hip_c3D[2];
                    mHIP_C = hip_c3D;
                }

                // Show angles
                CalcHumanJointAngles(Joints3Dekf, &mJointAngles, Twc);
                Rshank = mJointAngles.RShank;  Lshank = mJointAngles.LShank;
                RThigh = mJointAngles.RThigh;  LThigh = mJointAngles.LThigh;
                Rankle = mJointAngles.RAnkle;  Lankle = mJointAngles.LAnkle;
                RKnee  = mJointAngles.RKnee;   LKnee  = mJointAngles.LKnee;
                RFoot  = mJointAngles.RFoot;   LFoot  = mJointAngles.LFoot;
                Rftp   = mJointAngles.RFTP;    Lftp   = mJointAngles.LFTP;
            }
        }

        ///--------- display openpose  ---------//
        if(mbHumanPose){
            if (mpOpDetector->mlRenderPoseImage.size()>0){
                cv::Mat OpShow = mpOpDetector->mlRenderPoseImage.front();
                /// OPENCV IMSHOW
                //cv::imshow("Openpose", OpShow);
                //cv::waitKey(1);
                /// OPENGL DISPLAY
                pangolin::GlTexture imageTextureSkeleton(OpShow.cols,OpShow.rows,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
                imageTextureSkeleton.Upload(OpShow.data,GL_BGR,GL_UNSIGNED_BYTE);
                d_img_op.Activate();
                glColor3f(1.0,1.0,1.0);
                imageTextureSkeleton.RenderToViewportFlipY();
            }
        }



        ///--------- display skeleton of lower limb from front-view & side-view ---------//
        if(mbHumanPose) {
            if (mpOpDetector->mvJoints3DEKF.size() > 0) {
                // Skeleton2D image size: im.rows*3
                // Front view
                pangolin::GlTexture imageTextureFrontSkel(mWindowSizeY/3, mWindowSizeX, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
                cv::Mat Joints3Dekf = mpOpDetector->mvJoints3DEKF.back();
                std::vector<cv::Mat> mvJoints3DEKF = mpOpDetector->mvJoints3DEKF;
                cv::Mat SkelFrontView = DrawSkel2DView(Joints3Dekf, cv::Size(mWindowSizeY/3, mWindowSizeX), true);
                ///Draw2Dtrj(mvJoints3DEKF, SkelFrontView, true, mTrjHistory);
                imageTextureFrontSkel.Upload(SkelFrontView.data, GL_BGR, GL_UNSIGNED_BYTE);
                d_img_frontSkel.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTextureFrontSkel.RenderToViewportFlipY();
                // Side view
                pangolin::GlTexture imageTextureSideSkel(mWindowSizeY/3, mWindowSizeX, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
                cv::Mat SkelSideView = DrawSkel2DView(Joints3Dekf, cv::Size(mWindowSizeY/3, mWindowSizeX), false);
                ///Draw2Dtrj(mvJoints3DEKF, SkelSideView, false, mTrjHistory);
                imageTextureSideSkel.Upload(SkelSideView.data, GL_BGR, GL_UNSIGNED_BYTE);
                d_img_sideSkel.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTextureSideSkel.RenderToViewportFlipY();
            }
        }

        ///--------- Finish update OPENGL GUI in this frame ------------///
        pangolin::FinishFrame();

        ///--------- calculate wait time according to framerate ---------//
        const auto now = std::chrono::high_resolution_clock::now();
        const auto totalTimeSec =
                (double) std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_begin).count()
                * 1e-6;
        const auto message = "Visualisation time for each frame: "
                             + std::to_string(totalTimeSec - lastTime) + " milliseconds.";

        double wait_ms = (mT + lastTime - totalTimeSec);
        lastTime = totalTimeSec;
        if ( wait_ms > 1.0)
            cv::waitKey((int)wait_ms);
        else
            cv::waitKey(1);


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
    glColor3f(0.2f, 0.2f, 0.f);

    for ( int i=0; i < Joints3D.cols; i++){
        // No face
        if (Joints3D.at<cv::Vec3f>(i)[2] > 0 && (i < 15 || i > 18))
            glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(i)[0]
                    + Twc.m[4] * Joints3D.at<cv::Vec3f>(i)[1]
                    + Twc.m[8] * Joints3D.at<cv::Vec3f>(i)[2],
                       Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(i)[0]
                    + Twc.m[5] * Joints3D.at<cv::Vec3f>(i)[1]
                    + Twc.m[9] * Joints3D.at<cv::Vec3f>(i)[2],
                       Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(i)[0]
                    + Twc.m[6] * Joints3D.at<cv::Vec3f>(i)[1]
                    + Twc.m[10] * Joints3D.at<cv::Vec3f>(i)[2]);
    }
    glEnd();


    glLineWidth(5);
    glColor3f(0.0,0.0,0.8);
    int links[2][20] = {{0, 2, 2, 4, 5, 5, 7, 8 ,8 ,8,10,10,22,23,24,13,13,19,20,21},
                            {1, 1, 3, 3, 1, 6, 6, 1, 9,12, 9,11,11,11,11,12,14,14,14,14}};
    int linkNum = 20;
    glBegin(GL_LINES);
    for(int i=0; i<linkNum; i++){
        int p1 = links[0][i];
        int p2 = links[1][i];
        if (Joints3D.at<cv::Vec3f>(p1)[2] > 0 && Joints3D.at<cv::Vec3f>(p2)[2] > 0){
            glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p1)[2],
                       Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p1)[2],
                       Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p1)[2]);
            glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p2)[2],
                       Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p2)[2],
                       Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p2)[2]);
        }
    }
    glEnd();
}

void Viewer::Draw3DLowerJoints(cv::Mat Joints3D, int mode){
    // Draw point and links the 3D point clound
    glLineWidth(5);
    if (mode == 0)
        glColor3f(0.0,0.0,0.8);
    else
        glColor3f(0.0,0.8,0.5);

    int links[2][12] = {{8 ,8,10,10,22,23,24,13,13,19,20,21},
                        {9,12, 9,11,11,11,11,12,14,14,14,14}};
    int linkNum = 12;
    glBegin(GL_LINES);
    for(int i=0; i<linkNum; i++){
        int p1 = links[0][i];
        int p2 = links[1][i];
        if (Joints3D.at<cv::Vec3f>(p1)[2] > 0 && Joints3D.at<cv::Vec3f>(p2)[2] > 0){
            if (mode == 0){
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p1)[2],
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p1)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p1)[2]);
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p2)[2],
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p2)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p2)[2]);
            }
            else if (mode == 1){
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p1)[2] + 0.5,
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p1)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p1)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p1)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p1)[2]);
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[4] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[8] * Joints3D.at<cv::Vec3f>(p2)[2] + 0.5,
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[5] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[9] * Joints3D.at<cv::Vec3f>(p2)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(p2)[0] + Twc.m[6] * Joints3D.at<cv::Vec3f>(p2)[1] + Twc.m[10] * Joints3D.at<cv::Vec3f>(p2)[2]);

            }
        }
    }
    glEnd();

    glPointSize(50);
    glBegin(GL_POINTS);
    glColor3f(0.2f, 0.2f, 0.f);
    for ( int i=0; i < Joints3D.cols; i++){

        // No face
        if (Joints3D.at<cv::Vec3f>(i)[2] > 7 && (i < 15 || i > 18)){
            if (mode == 0){
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[4] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[8] * Joints3D.at<cv::Vec3f>(i)[2],
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[5] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[9] * Joints3D.at<cv::Vec3f>(i)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[6] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[10] * Joints3D.at<cv::Vec3f>(i)[2]);
            }
            else if (mode == 1){
                glVertex3f(Twc.m[12] + Twc.m[0] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[4] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[8] * Joints3D.at<cv::Vec3f>(i)[2] + 0.5,
                           Twc.m[13] + Twc.m[1] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[5] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[9] * Joints3D.at<cv::Vec3f>(i)[2],
                           Twc.m[14] + Twc.m[2] * Joints3D.at<cv::Vec3f>(i)[0]
                           + Twc.m[6] * Joints3D.at<cv::Vec3f>(i)[1]
                           + Twc.m[10] * Joints3D.at<cv::Vec3f>(i)[2]);
            }
        }
    }
    glEnd();

}

void Viewer::Draw2DHumanLoc(cv::Mat Joints3D){
    cv::Vec3f HIP_C = Joints3D.at<cv::Vec3f>(8);
    cv::Vec3f HIP_R = Joints3D.at<cv::Vec3f>(9);
    cv::Vec3f HIP_L = Joints3D.at<cv::Vec3f>(12);

    //Calculate Human body coordinate system in 2D and 3D
    ///need to be modified
    cv::Mat HBCoord3D = CalcHumanBodyCoord(HIP_R, HIP_C, HIP_L);
    cv::Mat HBCoord2D_TopView = cv::Mat(2,2, CV_32FC1);
    HBCoord2D_TopView.at<float>(0,0) = HBCoord3D.at<float>(0,0);
    HBCoord2D_TopView.at<float>(1,1) = HBCoord3D.at<float>(2,2);
    HBCoord2D_TopView.at<float>(0,1) = HBCoord3D.at<float>(0,2);
    HBCoord2D_TopView.at<float>(1,0) = HBCoord3D.at<float>(2,0);

    double x2D = Twc.m[12] + Twc.m[0] * HIP_C(0) + Twc.m[4] * HIP_C(1) + Twc.m[8] * HIP_C(2);
    double z2D = Twc.m[14] + Twc.m[2] * HIP_C(0) + Twc.m[6] * HIP_C(1) + Twc.m[10] * HIP_C(2);
    // Location
    glPointSize(100.0/abs(mViewpointY));
    glBegin(GL_POINTS);
    glColor3f(0.8f, 0.0f, 0.8f);
    glVertex3f(x2D, 0.0, z2D);
    glEnd();

    // Orientation
    glLineWidth(25.0/abs(mViewpointY));
    glColor3f(1.f,0.f,0.f);
    glBegin(GL_LINES);
    glVertex3f(x2D, 0.0, z2D);
    glVertex3f(x2D + HBCoord3D.at<float>(0,2)*0.5, 0.0, z2D + HBCoord3D.at<float>(2,2)*0.5);
    glEnd();
    glLineWidth(25.0/abs(mViewpointY));
    glColor3f(0.f,0.f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(x2D, 0.0, z2D);
    glVertex3f(x2D + HBCoord3D.at<float>(0,0)*0.5, 0.0, z2D + HBCoord3D.at<float>(2,0)*0.5);
    glEnd();
}

void Viewer::Draw2DCamLoc(pangolin::OpenGlMatrix &Twc){
    double CamLocX = Twc.m[12];
    double CamLocZ = Twc.m[14];

    double xx = Twc.m[0], xz = Twc.m[2], zx = Twc.m[8], zz = Twc.m[10];
    xx = xx / (sqrt(xx*xx + xz*xz) + 1e-23);
    xz = xz / (sqrt(xx*xx + xz*xz) + 1e-23);
    zx = zx / (sqrt(zz*zz + zx*zx) + 1e-23);
    zz = zz / (sqrt(zz*zz + zx*zx) + 1e-23);


    glLineWidth(25.0/abs(mViewpointY));
    glColor3f(0.f,1.0f,0.f);
    glBegin(GL_LINES);
    glVertex3f(CamLocX, 0.0, CamLocZ);
    glVertex3f(CamLocX+(xx+zx)*mCameraSize*2, 0.0, CamLocZ+(zz+xz)*mCameraSize*2);
    glVertex3f(CamLocX, 0.0, CamLocZ);
    glVertex3f(CamLocX-(xx-zx)*mCameraSize*2, 0.0, CamLocZ+(zz-xz)*mCameraSize*2);
    glVertex3f(CamLocX+(xx+zx)*mCameraSize*2, 0.0, CamLocZ+(zz+xz)*mCameraSize*2);
    glVertex3f(CamLocX-(xx-zx)*mCameraSize*2, 0.0, CamLocZ+(zz-xz)*mCameraSize*2);
    glEnd();


}

void Viewer::Draw3Dtrj(std::vector<cv::Mat>Joints3D, int N_history){
    glLineWidth(2);
    glColor3f(0.8,0.0,0.8);
    int pLowerLimb[13] = {8,9,10,11,12,13,14,19,20,21,22,23,24};
    int N_store = Joints3D.size();
    cv::Mat Joints_now, Joints_last;

    glBegin(GL_LINES);
    if (N_store >= N_history){ // current data > max plot trajectory data
        for (std::vector<cv::Mat>::iterator it = Joints3D.end() -1; it != Joints3D.end() - N_history; it = it - 1){
            if (it == Joints3D.end()-1){ //t=1
                Joints_last = *it;
            }
            else{ // t>1
                Joints_now = *it;
                for (int i = 0; i < 13; i++){
                    cv::Vec3f point_now, point_last;
                    point_now = Joints_now.at<cv::Vec3f>(pLowerLimb[i]);
                    point_last = Joints_last.at<cv::Vec3f>(pLowerLimb[i]);
                    if (point_now[2] > 0 && point_last[2]>0){  // These two points are valid
                        // Map points from camera coordinates to world coordinates
                        glVertex3f(Twc.m[12] + Twc.m[0] * point_now[0] + Twc.m[4] *  point_now[1] + Twc.m[8]  * point_now[2],
                                   Twc.m[13] + Twc.m[1] * point_now[0] + Twc.m[5] *  point_now[1] + Twc.m[9]  * point_now[2],
                                   Twc.m[14] + Twc.m[2] * point_now[0] + Twc.m[6] *  point_now[1] + Twc.m[10] * point_now[2]);
                        glVertex3f(Twc.m[12] + Twc.m[0] * point_last[0] + Twc.m[4] * point_last[1] + Twc.m[8] * point_last[2],
                                   Twc.m[13] + Twc.m[1] * point_last[0] + Twc.m[5] * point_last[1] + Twc.m[9] * point_last[2],
                                   Twc.m[14] + Twc.m[2] * point_last[0] + Twc.m[6] * point_last[1] + Twc.m[10]* point_last[2]);
                    }
                }
                Joints_last = *it;
            }
        }
    }
    else{ // current data < max plot trajectory data
        for (std::vector<cv::Mat>::iterator it = Joints3D.end()-1; it == Joints3D.begin(); it = it-1){
            if (it == Joints3D.end()-1){ //t=1
                Joints_last = *it;
            }
            else{ // t>1
                Joints_now = *it;
                for (int i = 0; i < 13; i++){
                    cv::Vec3f point_now, point_last;
                    point_now = Joints_now.at<cv::Vec3f>(pLowerLimb[i]);
                    point_last = Joints_last.at<cv::Vec3f>(pLowerLimb[i]);
                    if (point_now[2] > 0 && point_last[2]>0){  // These two points are valid
                        // Map points from camera coordinates to world coordinates
                        glVertex3f(Twc.m[12] + Twc.m[0] * point_now[0] + Twc.m[4] *  point_now[1] + Twc.m[8]  * point_now[2],
                                   Twc.m[13] + Twc.m[1] * point_now[0] + Twc.m[5] *  point_now[1] + Twc.m[9]  * point_now[2],
                                   Twc.m[14] + Twc.m[2] * point_now[0] + Twc.m[6] *  point_now[1] + Twc.m[10] * point_now[2]);
                        glVertex3f(Twc.m[12] + Twc.m[0] * point_last[0] + Twc.m[4] * point_last[1] + Twc.m[8] * point_last[2],
                                   Twc.m[13] + Twc.m[1] * point_last[0] + Twc.m[5] * point_last[1] + Twc.m[9] * point_last[2],
                                   Twc.m[14] + Twc.m[2] * point_last[0] + Twc.m[6] * point_last[1] + Twc.m[10]* point_last[2]);
                    }
                }
                Joints_last = *it;
            }
        }
    }
    glEnd();
}

void Viewer::Draw2Dtrj(std::vector<cv::Mat>Joints3D, cv::Mat& Img, bool FrontViewFlag, int N_history){
    /// 3D Lower Limb (pixel): 0.8m (width) x 1.2m (height)
    double HeigthPixel = Img.rows * 0.9;
    int floorBound = (int)(Img.rows * 0.9);
    double WidthPixel = Img.cols * 0.9;

    /// Determine the visulisation matrix: front view --> I, side view x<->z
    cv::Mat ViewCoord;
    if (FrontViewFlag){
        cv::Mat BufCoord= cv::Mat::eye(3, 3, CV_32FC1);
        BufCoord.copyTo(ViewCoord);
    }
    else{
        float tmp_showCoord[][3]={0,0,1,0,1,0,1,0,0};
        cv::Mat BufCoord(3, 3, CV_32FC1,(void *)tmp_showCoord, cv::Mat::AUTO_STEP);
        BufCoord.copyTo(ViewCoord);
    }

    int pLowerLimb[13] = {8,9,10,11,12,13,14,19,20,21,22,23,24};
    cv::Mat Joints_now, Joints_last;
    cv::Mat MatBuf;

    int N_size = Joints3D.size(), N_valid;
    if (N_size <= N_history)
        N_valid = N_size;
    else
        N_valid = N_history;

    for (std::vector<cv::Mat>::iterator it = Joints3D.end()-1; it != Joints3D.end() - N_valid ; it = it -1){
        if (it == Joints3D.end() - 1){ //t=1
            /// Calculate normalized 3D Joints
            MatBuf = *it;
            cv::Mat MatCpy = MatBuf.clone();
            // Build human body coordiantes
            cv::Mat HBcoord = CalcHumanBodyCoord(MatCpy.at<cv::Vec3f>(9), MatCpy.at<cv::Vec3f>(8), MatCpy.at<cv::Vec3f>(12));
            cv::Mat Rot = ViewCoord*HBcoord.t();
            // Remove rotation and translation of 3D joints
            cv::Mat JointsRST(25, 3, CV_32FC1, (void *)MatCpy.data, cv::Mat::AUTO_STEP);  //25*1*3 ==> 25*3*1
            Joints_last = JointsRST*Rot.t();
        }
        else{ // t>1
            /// Calculate normalized 3D Joints
            MatBuf = *it;
            cv::Mat MatCpy = MatBuf.clone();
            // Build human body coordiantes
            cv::Mat HBcoord = CalcHumanBodyCoord(MatCpy.at<cv::Vec3f>(9), MatCpy.at<cv::Vec3f>(8), MatCpy.at<cv::Vec3f>(12));
            cv::Mat Rot = ViewCoord*HBcoord.t();
            // Remove rotation and translation of 3D joints
            cv::Mat JointsRST(25, 3, CV_32FC1, (void *)MatCpy.data, cv::Mat::AUTO_STEP);  //25*1*3 ==> 25*3*1

            Joints_now = JointsRST*Rot.t();
            double delta_x_now = Joints_now.at<cv::Vec3f>(8)[0];
            double delta_x_last = Joints_last.at<cv::Vec3f>(8)[0];

            for (int i = 0; i < 13; i++){
                cv::Vec3f point_now, point_last;
                point_now = Joints_now.at<cv::Vec3f>(pLowerLimb[i]);
                point_last = Joints_last.at<cv::Vec3f>(pLowerLimb[i]);
                cv::Point2d jStartPlot, jEndPlot;

                jStartPlot.x = (point_now[0] - delta_x_now) * WidthPixel/(0.8)+ Img.cols/2; // Mov the x-axis of hip center as the center of image, 0.8
                jEndPlot.x   = (point_last[0]- delta_x_last) * WidthPixel/(0.8)+ Img.cols/2; // Mov the x-axis of hip center as the center of image
                jStartPlot.y = floorBound - (mCamZ - point_now[1]) * HeigthPixel/1.2;
                jEndPlot.y =   floorBound - (mCamZ - point_last[1]) * HeigthPixel/1.2;
                if (jStartPlot.x > 0 && jEndPlot.x > 0 &&
                    jStartPlot.x < Img.cols && jEndPlot.x < Img.cols &&
                    jStartPlot.y > 0 && jEndPlot.y > 0 &&
                    jStartPlot.y < Img.rows && jEndPlot.y < Img.rows)
                    cv::line(Img, jStartPlot, jEndPlot, cv::Scalar(0, 255, 255), 5);
            }
            Joints_last = Joints_now;
        }
    }
}

double Viewer::AngleLink2Link(cv::Vec3f point1, cv::Vec3f point_mid, cv::Vec3f point2){
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
    alpha = alpha*180.0/3.1415;
    return alpha;
}

double Viewer::AngleLink2Plane(cv::Vec3f point3d, cv::Mat plane3d){
    /// Plane 3d cv::Vec3f --> 0: mid point, 1: point 1, 2: point 2
    /// Link: point - mid point of the plane
    double beta = 0.0;

    cv::Vec3f planeVec1 = plane3d.at<cv::Vec3f>(1) - plane3d.at<cv::Vec3f>(0);
    double NormVec1 = cv::norm(planeVec1);
    cv::Vec3f planeVec2 = plane3d.at<cv::Vec3f>(2) - plane3d.at<cv::Vec3f>(0);
    double NormVec2 = cv::norm(planeVec2);
    planeVec1 = planeVec1 / (NormVec1 + 1e-23);
    planeVec2 = planeVec2 / (NormVec2 + 1e-23);

    cv::Vec3f planeNormV = planeVec1.cross(planeVec2);
    beta = AngleLink2Link(point3d, plane3d.at<cv::Vec3f>(0), plane3d.at<cv::Vec3f>(0) + planeNormV);
    beta = beta*180.0/3.1415;
    return beta;
}

double Viewer::AnglePlane2Plane(cv::Mat plane1, cv::Mat plane2){
    double theta = 0.0;
    cv::Vec3f plane1_Vec1 = plane1.at<cv::Vec3f>(1) - plane1.at<cv::Vec3f>(0);
    plane1_Vec1 = plane1_Vec1 / (cv::norm(plane1_Vec1) + 1e-23);
    cv::Vec3f plane1_Vec2 = plane1.at<cv::Vec3f>(2) - plane1.at<cv::Vec3f>(0);
    plane1_Vec2 = plane1_Vec2 / (cv::norm(plane1_Vec2) + 1e-23);
    cv::Vec3f plane2_Vec1 = plane1.at<cv::Vec3f>(1) - plane1.at<cv::Vec3f>(0);
    plane2_Vec1 = plane2_Vec1 / (cv::norm(plane2_Vec1) + 1e-23);
    cv::Vec3f plane2_Vec2 = plane1.at<cv::Vec3f>(2) - plane1.at<cv::Vec3f>(0);
    plane2_Vec2 = plane2_Vec2 / (cv::norm(plane2_Vec2) + 1e-23);

    cv::Vec3f NormV1 = plane1_Vec1.cross(plane1_Vec2);
    cv::Vec3f NormV2 = plane2_Vec1.cross(plane2_Vec2);
    cv::Vec3f Mid; Mid[0] = 0.0; Mid[1]=0.0; Mid[2]=0.0;

    theta = AngleLink2Link(NormV1, Mid, NormV2);
    theta = theta*180.0/3.1415;
    return theta;
}

cv::Mat Viewer::DrawSkel2DView(cv::Mat Joints3D, cv::Size ImgSize, bool FrontViewFlag){
    /// ViewFlag: 0 --> front view ; 1 --> side view
    cv::Mat Skel2DView = cv::Mat::zeros(ImgSize, CV_8UC3); // + cv::Scalar(200,200,200);
    //Skel2DView = Draw2Dtrj(Joints3D, Skel2DView, FrontViewFlag);

    Skel2DView.rowRange(0,5) = cv::Scalar(0,0,200);
    Skel2DView.rowRange(Skel2DView.rows-5,Skel2DView.rows) = cv::Scalar(0,0,200);
    Skel2DView.colRange(0,5) = cv::Scalar(0,0,200);
    Skel2DView.rowRange(Skel2DView.rows-5,Skel2DView.rows) = cv::Scalar(0,0,200);

    /// Determine the visulisation matrix: front view --> I, side view x<->z
    cv::Mat ViewCoord;
    if (FrontViewFlag){
        cv::Mat BufCoord= cv::Mat::eye(3, 3, CV_32FC1);
        BufCoord.copyTo(ViewCoord);
    }
    else{
        float tmp_showCoord[][3]={0,0,1,0,1,0,1,0,0};
        cv::Mat BufCoord(3, 3, CV_32FC1,(void *)tmp_showCoord, cv::Mat::AUTO_STEP);
        BufCoord.copyTo(ViewCoord);
    }

    /// 3D Lower Limb (pixel): 0.8m (width) x 1.2m (height)
    double HeigthPixel = ImgSize.height * 0.9;
    int floorBound = (int)(ImgSize.height * 0.9);
    double WidthPixel = ImgSize.width * 0.9;

    /// Plot floor on the image
    Skel2DView.rowRange(floorBound,Skel2DView.rows) = cv::Scalar(200,200,200);

    /// Only if 7 main joints are captured
    if (Joints3D.at<cv::Vec3f>(8)[2] * Joints3D.at<cv::Vec3f>(9)[2] * Joints3D.at<cv::Vec3f>(12)[2]
        * Joints3D.at<cv::Vec3f>(10)[2] * Joints3D.at<cv::Vec3f>(13)[2]
        * Joints3D.at<cv::Vec3f>(11)[2] * Joints3D.at<cv::Vec3f>(14)[2]){
        // links of lower limb
        int links[2][12] = {{8 ,8,10,10,22,23,24,13,13,19,20,21},
                            {9,12, 9,11,11,11,11,12,14,14,14,14}};
        // Build human body coordiantes
        cv::Mat HBcoord = CalcHumanBodyCoord(Joints3D.at<cv::Vec3f>(9), Joints3D.at<cv::Vec3f>(8), Joints3D.at<cv::Vec3f>(12));
        cv::Mat Rot = ViewCoord*HBcoord.t();
        cv::Mat MatCpy = Joints3D.clone();
        // Calculate normalized 3D
        cv::Mat Joints3D_Mat(25, 3, CV_32FC1, (void *)MatCpy.data, cv::Mat::AUTO_STEP);
        Joints3D_Mat = Joints3D_Mat*Rot.t();

        double delta_x = Joints3D_Mat.at<cv::Vec3f>(8)[0];

        for (int i = 0; i < 12; i++){
            cv::Vec3f jStart, jEnd;
            jStart = Joints3D_Mat.at<cv::Vec3f>(links[0][i]);
            jEnd = Joints3D_Mat.at<cv::Vec3f>(links[1][i]);
            cv::Point2d jStartPlot, jEndPlot;
            ///When these two joints are both detected
            if (Joints3D.at<cv::Vec3f>(links[0][i])[2] > 0 && Joints3D.at<cv::Vec3f>(links[1][i])[2] > 0){
                jStartPlot.x = (jStart[0] - delta_x) * WidthPixel/(0.8)
                               + Skel2DView.cols/2; // Mov the x-axis of hip center as the center of image, 0.8
                jEndPlot.x   = (jEnd[0]   - delta_x) * WidthPixel/(0.8)
                               + Skel2DView.cols/2; // Mov the x-axis of hip center as the center of image
                if (jStartPlot.x < 0)
                    jStartPlot.x = 0;
                if (jStartPlot.x > Skel2DView.cols)
                    jStartPlot.x = Skel2DView.cols;
                if (jEndPlot.x < 0)
                    jEndPlot.x = 0;
                if (jEndPlot.x > Skel2DView.cols)
                    jEndPlot.x = Skel2DView.cols;

                jStartPlot.y = floorBound - (mCamZ - jStart[1]) * HeigthPixel/1.2;
                jEndPlot.y =   floorBound - (mCamZ - jEnd[1]) * HeigthPixel/1.2;
                //jStartPlot.y = SkelFrontView.rows - abs(jStart[1]-mCamZ) * HeigthPixel;
                //jEndPlot.y =   SkelFrontView.rows - abs(jEnd[1]-mCamZ)   * HeigthPixel;
                if (jStartPlot.y < 0)
                    jStartPlot.y = 0;
                if (jStartPlot.y > Skel2DView.rows)
                    jStartPlot.y = Skel2DView.rows;
                if (jEndPlot.y < 0)
                    jEndPlot.y = 0;
                if (jEndPlot.y > Skel2DView.rows)
                    jEndPlot.y = Skel2DView.rows;
                cv::circle(Skel2DView, jStartPlot, 4, cv::Scalar(0,0,0), 5);
                cv::circle(Skel2DView, jEndPlot, 4, cv::Scalar(0,0,0), 5);
                cv::line(Skel2DView, jStartPlot, jEndPlot, cv::Scalar(255-i*10, i*20, 255-i*20), 10);
            }
        }
    }

    return Skel2DView;
}

cv::Mat Viewer::CalcHumanBodyCoord(cv::Vec3f HIP_R, cv::Vec3f HIP_C, cv::Vec3f HIP_L){
    cv::Mat HBcoord = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Vec3f v1, v2, axis_z, axis_x;
    cv::Vec3f axis_y(0.f, 1.f, 0.f);

    v1 = HIP_R - HIP_C;  v1 = v1 / (cv::norm(v1) + 1e-23);
    v2 = HIP_L - HIP_R;  v2 = v2 / (cv::norm(v2) + 1e-23);
    axis_z = v2.cross(axis_y);       axis_z = axis_z / (cv::norm(axis_z) + 1e-23);
    if (axis_z[2] > 0)
        axis_z = - axis_z;
    axis_x = axis_y.cross(axis_z);   axis_x = axis_x / (cv::norm(axis_x) + 1e-23);
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            if (j == 0)
                HBcoord.at<float>(i,j) = axis_x[i];
            if (j == 1)
                HBcoord.at<float>(i,j) = axis_y[i];
            if (j == 2)
                HBcoord.at<float>(i,j) = axis_z[i];
        }
    }

    return HBcoord;
}

double Viewer::CalcLinkLength(cv::Vec3f point1, cv::Vec3f point2){
    double L = 0;

    double squareSum = pow(point1[0]-point2[0],2) + pow(point1[1]-point2[1],2) + pow(point1[2]-point2[2],2);
    L = sqrt(squareSum);

    return L;
}

void Viewer::CalcHumanJointAngles(cv::Mat Joints3D, struct HumanJointAngles *mJointAngles, pangolin::OpenGlMatrix &Twc){
    cv::Mat SkelCam = Joints3D.clone();
    cv::Mat SkelWrd(SkelCam.rows, SkelCam.cols, CV_32FC3);
    // Show angles
    /// Knee angles and ankle angles are angles between two links
    // Right knee angle
    if (SkelCam.at<cv::Vec3f>(HIP_R)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(KNEE_R)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(ANKLE_R)[2] > 0) {
        mJointAngles->RKnee = AngleLink2Link(SkelCam.at<cv::Vec3f>(HIP_R),SkelCam.at<cv::Vec3f>(KNEE_R),SkelCam.at<cv::Vec3f>(ANKLE_R));
        if (mJointAngles->RKnee < 90)
            mJointAngles->RKnee = 180 - mJointAngles->RKnee;
    }
    // Left knee angle
    if (SkelCam.at<cv::Vec3f>(HIP_L)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(KNEE_L)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(ANKLE_L)[2] > 0) {
        mJointAngles->LKnee = AngleLink2Link(SkelCam.at<cv::Vec3f>(HIP_L),SkelCam.at<cv::Vec3f>(KNEE_L),SkelCam.at<cv::Vec3f>(ANKLE_L));
        if (mJointAngles->LKnee < 90)
            mJointAngles->LKnee = 180 - mJointAngles->LKnee;
    }
    // Right ankle angle
    if (SkelCam.at<cv::Vec3f>(TOE_IN_R)[2] > 0 && SkelCam.at<cv::Vec3f>(TOE_OUT_R)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(KNEE_R)[2] > 0 && SkelCam.at<cv::Vec3f>(ANKLE_R)[2] > 0){
        cv::Vec3f FootR_mid = (SkelCam.at<cv::Vec3f>(TOE_IN_R) + SkelCam.at<cv::Vec3f>(TOE_OUT_R));
        mJointAngles->RAnkle = AngleLink2Link(SkelCam.at<cv::Vec3f>(KNEE_R),SkelCam.at<cv::Vec3f>(ANKLE_R),FootR_mid*0.5);
    }
    // Left ankle angle
    if (SkelCam.at<cv::Vec3f>(TOE_IN_L)[2] > 0 && SkelCam.at<cv::Vec3f>(TOE_OUT_L)[2] > 0 &&
            SkelCam.at<cv::Vec3f>(KNEE_L)[2] > 0 && SkelCam.at<cv::Vec3f>(ANKLE_L)[2] > 0){
        cv::Vec3f FootL_mid = (SkelCam.at<cv::Vec3f>(TOE_IN_L) + SkelCam.at<cv::Vec3f>(TOE_OUT_L));
        mJointAngles->LAnkle = AngleLink2Link(SkelCam.at<cv::Vec3f>(KNEE_L),SkelCam.at<cv::Vec3f>(ANKLE_L),FootL_mid*0.5);
    }
    /** Thigh, shank, and foot angles are angle between link and floor normal plane, where the floor normal plane is set as [0 1 0] in {W}.
     * That means the camera pose is parallel to the floor ground in the first frame
     * Next, we need to project the Joints in Camera Space into the World coordinates */
     for (int i = 0; i < Joints3D.cols; i++){
         SkelWrd.at<cv::Vec3f>(i)[0] = Twc.m[12] + Twc.m[0] * SkelCam.at<cv::Vec3f>(i)[0]
                                     + Twc.m[4] * SkelCam.at<cv::Vec3f>(i)[1]
                                     + Twc.m[8] * SkelCam.at<cv::Vec3f>(i)[2];
         SkelWrd.at<cv::Vec3f>(i)[1] = Twc.m[13] + Twc.m[1] * SkelCam.at<cv::Vec3f>(i)[0]
                                     + Twc.m[5] * SkelCam.at<cv::Vec3f>(i)[1]
                                     + Twc.m[9] * SkelCam.at<cv::Vec3f>(i)[2];
         SkelWrd.at<cv::Vec3f>(i)[2] = Twc.m[14] + Twc.m[2] * SkelCam.at<cv::Vec3f>(i)[0]
                                     + Twc.m[6] * SkelCam.at<cv::Vec3f>(i)[1]
                                     + Twc.m[10] * SkelCam.at<cv::Vec3f>(i)[2];
     }
     cv::Vec3f FloorNorm; FloorNorm[0]=0.0; FloorNorm[1]=-1.0; FloorNorm[2]=0.0;
    // Right thigh angle
    if (SkelCam.at<cv::Vec3f>(HIP_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(KNEE_R)[2] > 0) {
        mJointAngles->RThigh = AngleLink2Link(SkelWrd.at<cv::Vec3f>(KNEE_R),SkelWrd.at<cv::Vec3f>(HIP_R),SkelWrd.at<cv::Vec3f>(HIP_R)+FloorNorm);
        if (mJointAngles->RThigh < 90)
            mJointAngles->RThigh = 180 - mJointAngles->RThigh;
    }
    // Left thigh angle
    if (SkelCam.at<cv::Vec3f>(HIP_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(KNEE_L)[2] > 0) {
        mJointAngles->LThigh = AngleLink2Link(SkelWrd.at<cv::Vec3f>(KNEE_L),SkelWrd.at<cv::Vec3f>(HIP_L),SkelWrd.at<cv::Vec3f>(HIP_L)+FloorNorm);
        if (mJointAngles->LThigh < 90)
            mJointAngles->LThigh = 180 - mJointAngles->LThigh;
    }
    // Right shank angle
    if (SkelCam.at<cv::Vec3f>(KNEE_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(ANKLE_R)[2] > 0) {
        mJointAngles->RShank = AngleLink2Link(SkelWrd.at<cv::Vec3f>(ANKLE_R),SkelWrd.at<cv::Vec3f>(KNEE_R),SkelWrd.at<cv::Vec3f>(KNEE_R)+FloorNorm);
        if (mJointAngles->RShank < 90)
            mJointAngles->RShank = 180 - mJointAngles->RShank;
    }
    // Left shank angle
    if (SkelCam.at<cv::Vec3f>(KNEE_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(ANKLE_L)[2] > 0) {
        mJointAngles->LShank = AngleLink2Link(SkelWrd.at<cv::Vec3f>(ANKLE_L),SkelWrd.at<cv::Vec3f>(KNEE_L),SkelWrd.at<cv::Vec3f>(KNEE_L)+FloorNorm);
        if (mJointAngles->LShank < 90)
            mJointAngles->LShank = 180 - mJointAngles->LShank;
    }
    // Right foot angle
    if (SkelCam.at<cv::Vec3f>(ANKLE_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_IN_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_OUT_R)[2] > 0) {
        cv::Vec3f FootMid = SkelWrd.at<cv::Vec3f>(TOE_IN_R) + SkelWrd.at<cv::Vec3f>(TOE_OUT_R);
        mJointAngles->RFoot = AngleLink2Link(FootMid,SkelWrd.at<cv::Vec3f>(ANKLE_R),SkelWrd.at<cv::Vec3f>(ANKLE_R)+FloorNorm);
    }
    // Left foot angle
    if (SkelCam.at<cv::Vec3f>(ANKLE_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_IN_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_OUT_L)[2] > 0) {
        cv::Vec3f FootMid = SkelWrd.at<cv::Vec3f>(TOE_IN_L) + SkelWrd.at<cv::Vec3f>(TOE_OUT_L);
        mJointAngles->LFoot = AngleLink2Link(FootMid,SkelWrd.at<cv::Vec3f>(ANKLE_L),SkelWrd.at<cv::Vec3f>(ANKLE_L)+FloorNorm);
    }
    /** Foot progression angle, the angle between forward path and foot link in the floor plane */
    cv::Mat HBCoord = CalcHumanBodyCoord(SkelWrd.at<cv::Vec3f>(HIP_R), SkelWrd.at<cv::Vec3f>(HIP_C), SkelWrd.at<cv::Vec3f>(HIP_L));
    cv::Vec3f ForwardDirect;
    ForwardDirect[0] = HBCoord.at<float>(0,0);
    ForwardDirect[1] = 0.0;
    ForwardDirect[2] = HBCoord.at<float>(2,0);
    // Right FTP
    if (SkelCam.at<cv::Vec3f>(ANKLE_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_IN_R)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_OUT_R)[2] > 0) {
        cv::Vec3f FootMid = SkelWrd.at<cv::Vec3f>(TOE_OUT_R); FootMid[1] = 0.0;
        cv::Vec3f AnkleR = SkelCam.at<cv::Vec3f>(ANKLE_R); AnkleR[1] = 0.0;
        mJointAngles->RFTP = AngleLink2Link(FootMid,AnkleR,AnkleR+ForwardDirect);
        if (mJointAngles->RFTP > 90)
            mJointAngles->RFTP = 180 - mJointAngles->RFTP ;
    }
    // Left FTP
    if (SkelCam.at<cv::Vec3f>(ANKLE_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_IN_L)[2] > 0 &&
        SkelCam.at<cv::Vec3f>(TOE_OUT_L)[2] > 0) {
        cv::Vec3f FootMid = SkelWrd.at<cv::Vec3f>(TOE_OUT_L); FootMid[1] = 0.0;
        cv::Vec3f AnkleL = SkelCam.at<cv::Vec3f>(ANKLE_L); AnkleL[1] = 0.0;
        mJointAngles->LFTP = 180 - AngleLink2Link(FootMid,AnkleL,AnkleL+ForwardDirect);
        if (mJointAngles->LFTP > 90)
            mJointAngles->LFTP = 180 - mJointAngles->LFTP ;
    }
}

void Viewer::RequestFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop(){
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped(){
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop(){
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

void Viewer::Release(){
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
