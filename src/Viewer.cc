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

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, ArucoDetector *pArucoDetector, const string &strSettingPath, bool bReuse):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),mpArucoDetector(pArucoDetector),
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

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);
    pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuShutDown("menu.Shut Down",false,false);
    pangolin::Var<double> showPosX("menu.X(red)", 0);
    pangolin::Var<double> showPosY("menu.Y(green)", 0);
    pangolin::Var<double> showPosZ("menu.Z(blue)", 0);
    pangolin::Var<int> NumberOfAruco("menu.noAruco",0, 0, 6);

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


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = mbReuse;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //Draw the Grid every frame
        float xmin=-10.0, xmax=10.0, dx=0.5, x;
        float zmin=-10.0, zmax=10.0, dz=0.5, z;
        glBegin(GL_LINES);
        for(x=xmin; x<=xmax; x+=dx)
        {
            for(z=zmin; z<=zmax; z+=dz)
            {
                glVertex3f(x, 1.0, zmin);
                glVertex3f(x, 1.0, zmax);
                glVertex3f(xmin, 1.0, z);
                glVertex3f(xmax, 1.0, z);
            }
        }
        glEnd();

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

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

        showPosX=Twc.m[4];
        showPosY=Twc.m[8];
        showPosZ=Twc.m[12];
        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        // Draw detected aruco marker on im
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

            cv::aruco::drawDetectedMarkers(im, corners, ids);
            for (unsigned int i = 0; i < ids.size(); i++){
                if (estimatePose){
                    cv::aruco::drawAxis(im, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
            }
        }
        cv::imshow("ORB-SLAM2: Current Frame",im);

        cv::waitKey(mT);

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
