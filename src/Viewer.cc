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

    mbReuse = bReuse;
    mbHumanPose = bHumanPose;
    mbARUCODetect = bARUCODetect;
}

void Viewer::Run()
{
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",mWindowSizeY,mWindowSizeX);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,0.1);
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
                pangolin::ProjectionMatrix(mWindowSizeY,mWindowSizeX,mViewpointF,mViewpointF,960,540,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.4, 1.0, 0.1, 0.7, -mWindowSizeY/mWindowSizeX)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_img_orb = pangolin::Display("orb_slam")
            .SetBounds(0.67, 1.0, 0.7, 1.0, mWindowSizeY/mWindowSizeX);
            //.SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_op = pangolin::Display("openpose_skeleton")
            .SetBounds(0.33, 0.66, 0.7,1.0, mWindowSizeY/mWindowSizeX);
            //.SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_frontSkel = pangolin::Display("frontview_skeleton")
            .SetBounds(0, 0.4, 0.1, 0.28)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& d_img_sideSkel = pangolin::Display("sideview_skeleton")
            .SetBounds(0, 0.4, 0.32, 0.5)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // Add Global coordinate system
    pangolin::Renderable tree;
    tree.Add( std::make_shared<pangolin::Axis>() );
    d_cam.SetDrawFunction([&](pangolin::View& view){
        view.Activate(s_cam);
        tree.Render();
    });


    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");
    //cv::namedWindow("Openpose");

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

        //--------- display ARUCO marker  ---------//
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

        //--------- display 3D human pose  ---------//
        if(mbHumanPose && mSensor == 2){
            if (mpOpDetector->mvJoints3DEKF.size()>0){
                cv::Mat Joints3Dekf = mpOpDetector->mvJoints3DEKF.back();
                Draw3DJoints(Joints3Dekf);
                std::vector<cv::Mat> mvJoints3DEKF = mpOpDetector->mvJoints3DEKF;
                Draw3Dtrj(mvJoints3DEKF, mTrjHistory);


                // Show distance to hip center
                cv::Vec3f hip_c = Joints3Dekf.at<cv::Vec3f>(8);
                if (hip_c[2] > 0){
                    distHipX = Twc.m[12] + hip_c[0]; distHipY = Twc.m[13] + hip_c[1]; distHipZ = Twc.m[14] + hip_c[2];
                }

                // Show angles
                // Knee R
                if (Joints3Dekf.at<cv::Vec3f>(9)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(10)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(11)[2] > 0) {
                    angleLegR = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(9),Joints3Dekf.at<cv::Vec3f>(10),Joints3Dekf.at<cv::Vec3f>(11));
                }
                // Knee L
                if (Joints3Dekf.at<cv::Vec3f>(12)[2] > 0 &&Joints3Dekf.at<cv::Vec3f>(13)[2] > 0 &&Joints3Dekf.at<cv::Vec3f>(14)[2] > 0) {
                    angleLegL = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(12),Joints3Dekf.at<cv::Vec3f>(13),Joints3Dekf.at<cv::Vec3f>(14));
                }
                // Foot R
                if (Joints3Dekf.at<cv::Vec3f>(10)[2] > 0 &&Joints3Dekf.at<cv::Vec3f>(11)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(22)[2] > 0 &&Joints3Dekf.at<cv::Vec3f>(23)[2] > 0){
                    cv::Vec3f FootR_mid = (Joints3Dekf.at<cv::Vec3f>(22) + Joints3Dekf.at<cv::Vec3f>(23));
                    angleFootR = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(10),Joints3Dekf.at<cv::Vec3f>(11),FootR_mid*0.5);
                }

                // Foot L
                if (Joints3Dekf.at<cv::Vec3f>(13)[2] > 0 && Joints3Dekf.at<cv::Vec3f>(14)[2] > 0 &&
                    Joints3Dekf.at<cv::Vec3f>(19)[2] > 0 &&Joints3Dekf.at<cv::Vec3f>(20)[2] > 0){
                    cv::Vec3f FootL_mid = (Joints3Dekf.at<cv::Vec3f>(19) + Joints3Dekf.at<cv::Vec3f>(20));
                    angleFootL = AnglePoint2Point(Joints3Dekf.at<cv::Vec3f>(13),Joints3Dekf.at<cv::Vec3f>(14),FootL_mid*0.5);
                }
            }
        }

        //--------- display 2D human pose  ---------//
        if(mbHumanPose){
            if (mpOpDetector->mlRenderPoseImage.size()>0){
                cv::Mat OpShow = mpOpDetector->mlRenderPoseImage.front();
                // display the image in opencv
                //cv::imshow("Openpose", OpShow);
                //cv::waitKey(1);

                // display the image in opengl
                pangolin::GlTexture imageTextureSkeleton(OpShow.cols,OpShow.rows,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
                imageTextureSkeleton.Upload(OpShow.data,GL_BGR,GL_UNSIGNED_BYTE);
                d_img_op.Activate();
                glColor3f(1.0,1.0,1.0);
                imageTextureSkeleton.RenderToViewportFlipY();
            }
        }

        //--------- display ORB-SLAM  ---------//
        // display ORB-SLAM in opencv
        //cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::Mat im = mpFrameDrawer->DrawFrame();
        // display the image in opengl
        pangolin::GlTexture imageTextureORB(im.cols,im.rows,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
        imageTextureORB.Upload(im.data,GL_BGR,GL_UNSIGNED_BYTE);
        d_img_orb.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTextureORB.RenderToViewportFlipY();

        //--------- display skeleton of lower limb from frontview  ---------//
        if(mbHumanPose && mSensor == 2) {
            if (mpOpDetector->mvJoints3DEKF.size() > 0) {
                pangolin::GlTexture imageTextureFrontSkel(im.cols, im.rows*3, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
                cv::Mat Joints3Dekf = mpOpDetector->mvJoints3DEKF.back();
                cv::Mat SkelFrontView = DrawSkel2DView(Joints3Dekf, cv::Size(im.cols, im.rows*3), true);
                //cv::Mat SkelFrontView = DrawSkelFrontView(Joints3Dekf, cv::Size(im.cols, im.rows*3));
                //cv::imwrite("Skel.jpg", SkelFrontView);
                imageTextureFrontSkel.Upload(SkelFrontView.data, GL_BGR, GL_UNSIGNED_BYTE);
                d_img_frontSkel.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTextureFrontSkel.RenderToViewportFlipY();

                pangolin::GlTexture imageTextureSideSkel(im.cols, im.rows*3, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
                cv::Mat SkelSideView = DrawSkel2DView(Joints3Dekf, cv::Size(im.cols, im.rows*3), false);
                imageTextureSideSkel.Upload(SkelSideView.data, GL_BGR, GL_UNSIGNED_BYTE);
                d_img_sideSkel.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTextureSideSkel.RenderToViewportFlipY();
            }
        }

        pangolin::FinishFrame();

        //--------- calculate wait time according to framerate ---------//
        const auto now = std::chrono::high_resolution_clock::now();
        const auto totalTimeSec =
                (double) std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_begin).count()
                * 1e-6;
        const auto message = "One frame time: "
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
    vector<cv::Mat> channels(3);
    split(Joints3D, channels);

    for ( int i=0; i < Joints3D.cols; i++){
        // No face
        if (channels[2].at<float>(0,i) > 0 && (i < 15 || i > 18))
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

void Viewer::Draw2Dtrj(std::vector<cv::Mat> Joints3D, cv::Mat Img, bool FrontViewFlag){

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

cv::Mat Viewer::DrawSkel2DView(cv::Mat Joints3D, cv::Size ImgSize, bool FrontViewFlag){
    /// ViewFlag: 0 --> front view ; 1 --> side view
    cv::Mat Skel2DView = cv::Mat::zeros(ImgSize, CV_8UC3) + cv::Scalar(255,255,255);
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
    std::vector<cv::Mat> FloorChannels(3);

    /// Plot floor on the image
    cv::Mat FloorC1 = cv::Mat::ones(ImgSize.height - floorBound , ImgSize.width, CV_8UC1);
    FloorC1.convertTo(FloorC1, CV_8UC1, 200); FloorC1.copyTo(FloorChannels[0]);
    cv::Mat FloorC2 = cv::Mat::ones(ImgSize.height - floorBound , ImgSize.width, CV_8UC1);
    FloorC2.convertTo(FloorC2, CV_8UC1, 200); FloorC2.copyTo(FloorChannels[1]);
    cv::Mat FloorC3 = cv::Mat::ones(ImgSize.height - floorBound , ImgSize.width, CV_8UC1);
    FloorC3.convertTo(FloorC3, CV_8UC1, 200); FloorC3.copyTo(FloorChannels[2]);
    cv::Mat FloorRender;
    cv::merge(FloorChannels, FloorRender);
    FloorRender.copyTo( Skel2DView.rowRange(floorBound,ImgSize.height));

    if (Joints3D.at<cv::Vec3f>(8)[2] * Joints3D.at<cv::Vec3f>(9)[2] * Joints3D.at<cv::Vec3f>(12)[2]
        * Joints3D.at<cv::Vec3f>(10)[2] * Joints3D.at<cv::Vec3f>(13)[2]
        * Joints3D.at<cv::Vec3f>(11)[2] * Joints3D.at<cv::Vec3f>(14)[2]){
        // links of lower limb
        int links[2][12] = {{8 ,8,10,10,22,23,24,13,13,19,20,21},
                            {9,12, 9,11,11,11,11,12,14,14,14,14}};
        // Build human body coordiantes
        cv::Mat HBcoord = CalcHumanBodyCoord(Joints3D.at<cv::Vec3f>(9), Joints3D.at<cv::Vec3f>(8), Joints3D.at<cv::Vec3f>(12));
        cv::Mat Rot = ViewCoord*HBcoord.t();
        cv::Mat Joints3D_copy = Joints3D.clone();
        // Calculate normalized 3D
        cv::Mat Joints3D_Mat(25, 3, CV_32FC1, (void *)Joints3D_copy.data, cv::Mat::AUTO_STEP);
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
