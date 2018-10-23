//
// Created by skaegy on 16/08/18.
//
#include <openpose/flags.hpp>
#include "DetectHumanPose.h"


using namespace std;
using namespace cv;
//using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

OpDetector::OpDetector(const string &strOpenposeSettingsFile, const bool bHumanPose, const int SensorMode){
    mbHumanPose = bHumanPose;
    cv::FileStorage fs(strOpenposeSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Unable to open openpose parameter file!" << endl;
    }

    // int
    mOPflag_logging_level = fs["logging_level"];
    mOPflag_scale_number = fs["scale_number"];
    mOPflag_num_gpu = fs["num_gpu"];
    mOPflag_num_gpu_start = fs["num_gpu_start"];
    mOPflag_op_tracking = fs["op_tracking"];
    mOPflag_number_people_max = fs["number_people_max"];
    mOPflag_part_to_show = fs["part_to_show"];

    // double
    mOPflag_scale_gap = fs["scale_gap"];
    mOPflag_alpha_pose = fs["alpha_pose"];
    mOPflag_render_threshold = fs["render_threshold"];

    // string
    const string var_model_pose = fs["model_pose"];
    mOPflag_model_pose = var_model_pose;
    const string var_model_folder = fs["model_folder"];
    mOPflag_model_folder = var_model_folder;
    const int net_resolution_row = fs["net_resolution_row"];
    const int net_resolution_col = fs["net_resolution_col"];
    mOPflag_net_resolution = to_string(net_resolution_row)+"x"+to_string(net_resolution_col);
    const int output_resolution_row = fs["output_resolution_row"];
    const int output_resolution_col = fs["output_resolution_col"];
    mOPflag_output_resolution = to_string(output_resolution_row)+"x"+to_string(output_resolution_col);

    mCam_fx = fs["Camera.fx"];
    mCam_fy = fs["Camera.fy"];
    mCam_ppx = fs["Camera.cx"];
    mCam_ppy = fs["Camera.cy"];
    mKF_wk = fs["KF.wk"];
    mKF_vk = fs["KF.vk"];
    mKF_pk = fs["KF.pk"];

    fs.release();
    mSensor = SensorMode;
}

void OpDetector::Run() {
    /* OP-Pose from image
    // ------------------------- OPENPOSE INITIALIZATION -------------------------
    // Step 1 - Set logging level
    // - 0 will output all the logging messages
    // - 255 will output nothing
    op::check(0 <= logging_level && logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority) logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    const auto outputSize = op::flagsToPoint(output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(net_resolution, "-1x368");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(model_pose);
    // Check no contradictory flags enabled
    if (alpha_pose < 0. || alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (scale_gap <= 0. && scale_gap > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                  __LINE__, __FUNCTION__, __FILE__);
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 3 - Initialize all required classes
    op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, scale_number, scale_gap);
    op::CvMatToOpInput cvMatToOpInput{poseModel};
    op::CvMatToOpOutput cvMatToOpOutput;
    op::PoseExtractorCaffe poseExtractorCaffe{poseModel, model_folder, num_gpu_start};
    op::PoseCpuRenderer poseRenderer{poseModel, (float) render_threshold, true, (float) alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat;
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();

    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec =
            (double) std::chrono::duration_cast<std::chrono::nanoseconds>(now - timerBegin).count()
            * 1e-9;
    const auto message = "OpenPose demo initialized. Total time: "
                         + std::to_string(totalTimeSec - lastTimeSec) + " seconds.";
    op::log(message, op::Priority::High);
    OpStandBy = true;
    */

    /* OP-Tracking
     * */
    op::check(0 <= mOPflag_logging_level && mOPflag_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority) mOPflag_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    // Applying user defined configuration
    // outputSize
    const auto outputSize = op::flagsToPoint(mOPflag_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(mOPflag_net_resolution, "-1x368");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(mOPflag_model_pose);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // >1 camera view?
    // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    const auto multipleView = false;
    // Enabling Google Logging
    const bool enableGoogleLogging = true;

    // Configuring OpenPose
    op::log("Configuring OpenPose...", op::Priority::High);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{
            !FLAGS_body_disable, netInputSize, outputSize, keypointScale, mOPflag_num_gpu, mOPflag_num_gpu_start,
            mOPflag_scale_number, (float)mOPflag_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
            poseModel, !FLAGS_disable_blending, (float)mOPflag_alpha_pose, (float)FLAGS_alpha_heatmap,
            mOPflag_part_to_show, mOPflag_model_folder, heatMapTypes, heatMapScale, FLAGS_part_candidates,
            (float)mOPflag_render_threshold, mOPflag_number_people_max, enableGoogleLogging};
    opWrapper.configure(wrapperStructPose);

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, mOPflag_op_tracking, FLAGS_ik_threads};
    opWrapper.configure(wrapperStructExtra);

    // Consumer (comment or use default argument to disable any output)
    const auto displayMode = op::DisplayMode::NoDisplay;
    const bool guiVerbose = false;
    const bool fullScreen = false;
    const op::WrapperStructOutput wrapperStructOutput{
            displayMode, guiVerbose, fullScreen, FLAGS_write_keypoint,
            op::stringToDataFormat(FLAGS_write_keypoint_format), FLAGS_write_json, FLAGS_write_coco_json,
            FLAGS_write_coco_foot_json, FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
            FLAGS_camera_fps, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_adam,
            FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
    opWrapper.configure(wrapperStructOutput);

    if (FLAGS_disable_multi_thread)
        opWrapper.disableMultiThreading();
    // Starting OpenPose
    op::log("Starting Openpose ...", op::Priority::High);
    opWrapper.start();
    OpStandBy = true;
    /* OP-Tracking */

    // ------------------------- 3D --> KALMAN FILTER INITIALIZATION -------------------------
    const int stateNum = 6;
    const int measureNum = 3;
    for (int i = 0; i < 25; i ++){
        KFs3D[i] = KFInitialization(stateNum, measureNum, mKF_wk, mKF_vk, mKF_pk);}
    //TODO: Skeleton tree for smooth
    InitHumanParams(&mHumanParams);

    while (!mbStopped ) {
        if (mbHumanPose && mlLoadImage.size() > 0) {
            mMutexColorIm.lock();
            cv::Mat inputImage = mlLoadImage.front();
            double timestamp = mlLoadTimestamp.front();
            mMutexColorIm.unlock();

            auto datumProcessed = opWrapper.emplaceAndPop(inputImage);
            cv::Mat joints2D;
            cv::Mat OutputImage;
            if (datumProcessed != nullptr)
            {
                const auto poseKeypoints = datumProcessed->at(0).poseKeypoints;
                OutputImage = datumProcessed->at(0).cvOutputData;
                joints2D = poseKeypoints.getConstCvMat();
            }



            /* OP-Pose from Image
            const op::Point<int> imageSize{inputImage.cols, inputImage.rows};
            // Step 2 - Get desired scale sizes
            std::vector<double> scaleInputToNetInputs;
            std::vector<op::Point<int>> netInputSizes;
            double scaleInputToOutput;
            op::Point<int> outputResolution;
            std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
                    = scaleAndSizeExtractor.extract(imageSize);
            // Step 3 - Format input image to OpenPose input and output formats
            const auto netInputArray = cvMatToOpInput.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
            auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);
            auto newoutputArray = outputArray;
            // Step 4 - Estimate poseKeypoints
            poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();

            // Step 6 - Render poseKeypoints
            //poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);
            // Step 7 - OpenPose output format to cv::Mat
            //auto OutputImage = opOutputToCvMat.formatToCvMat(outputArray);
            // Step 7: Extract most significant 2D joints and Compute 3D positions (if RGB-D)
            cv::Mat joints2D = poseKeypoints.getConstCvMat();
            */

            if (joints2D.rows > 0){
                /* OP-Pose from image
                cv::Mat keyJoint2D =GetInformPersonJoint(joints2D, render_threshold, cv::Size(inputImage.cols, inputImage.rows));

                auto newposeKeypoints = cvMatToOpOutput.createArray(keyJoint2D, scaleInputToOutput, outputResolution);
                poseRenderer.renderPose(newoutputArray, newposeKeypoints, scaleInputToOutput);
                auto newOutputImage = opOutputToCvMat.formatToCvMat(newoutputArray);
                */

                /* OP-Tracking */
                joints2D.copyTo(mJoints2D);
                /* OP-Tracking */
                //Plot2DJoints(mJoints2D, inputImage, render_threshold);

                mMutexOutputIm.lock();
                mlRenderPoseImage.push_front(OutputImage);
                if (mlRenderPoseImage.size() > 2)
                    mlRenderPoseImage.pop_back();
                mMutexOutputIm.unlock();

                // RGB-D (mSensor == 2)
                if (mlLoadDepth.size() > 0){
                    const auto timeBegin = std::chrono::high_resolution_clock::now();
                    /// Get depth value and map to 3D skeleton
                    mMutexDepthIm.lock();
                    cv::Mat inputDepth = mlLoadDepth.front();
                    mMutexDepthIm.unlock();
                    mMutexJoint.lock();
                    mJoints3D = Joints2Dto3D(mJoints2D, inputDepth, mOPflag_render_threshold);
                    mvJoints3Draw.push_back(mJoints3D);
                    mMutexJoint.unlock();

                    /// KALMAN SMOOTHER
                    mJoints3D_EKFsmooth = KFupdate(mJoints3D, stateNum, measureNum);

                    /// BODY PHYSICAL CONSTRAINTS
                    // Update the human body parameters during the tracking,
                    // which will be used as the follow physical constraints in the smoothing
                    UpdateHumanParams(mJoints3D_EKFsmooth, &mHumanParams);

                    /// Generate the mask according to the joints captured from openpose
                    mHumanMask = cv::Mat::ones(inputImage.size(),CV_8UC1);
                    mHumanMask.setTo(cv::Scalar(255));
                    //Joints2DSeg(inputImage, mJoints2D, mHumanMask);
                    Joints3DSeg(inputDepth,mJoints2D,mJoints3D_EKFsmooth,mHumanMask);
                    mMutexOutputIm.lock();
                    mlHumanMask.push_front(mHumanMask);
                    if (mlHumanMask.size() > 2)
                        mlHumanMask.pop_back();
                    mMutexOutputIm.unlock();

                    mMutexJoint.lock();
                    mvJoints3DEKF.push_back(mJoints3D_EKFsmooth);
                    mvTimestamp.push_back(timestamp);
                    mMutexJoint.unlock();

                    const auto now_time = std::chrono::high_resolution_clock::now();
                    const auto totalTimeMilSec =
                            (double) std::chrono::duration_cast<std::chrono::nanoseconds>(now_time - timeBegin).count()
                            * 1e-6;
                    //cout << "Skeleton process time: " << totalTimeMilSec << " (ms)" << endl;
                }
            }
            else{
                mMutexOutputIm.lock();
                mlRenderPoseImage.push_front(inputImage);
                if (mlRenderPoseImage.size() > 2)
                    mlRenderPoseImage.pop_back();
                mMutexOutputIm.unlock();
            }

            mFramecnt++;
        }
    }
}

void OpDetector::SetViewer(ORB_SLAM2::Viewer *pViewer){
    mpViewer = pViewer;
}

void OpDetector::OpLoadImageMonocular(const cv::Mat &im, const double &timestamp) {
    cv::Mat BufMat;
    im.copyTo(BufMat);
    mlLoadImage.push_front(BufMat);
    if (mlLoadImage.size()>1)
        mlLoadImage.pop_back();
    mlLoadTimestamp.push_front(timestamp);
    if (mlLoadTimestamp.size()>1)
        mlLoadTimestamp.pop_back();
}

void OpDetector::OpLoadImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
    mMutexColorIm.lock();
    cv::Mat BufRGB = imRGB.clone();
    mlLoadImage.push_front(BufRGB);
    if (mlLoadImage.size()>1)
        mlLoadImage.pop_back();
    mMutexColorIm.unlock();

    mMutexDepthIm.lock();
    cv::Mat BufDepth = imD.clone();
    mlLoadDepth.push_front(BufDepth);
    if (mlLoadDepth.size()>1)
        mlLoadDepth.pop_back();
    mlLoadTimestamp.push_front(timestamp);
    if (mlLoadTimestamp.size()>1)
        mlLoadTimestamp.pop_back();
    mMutexDepthIm.unlock();
}

cv::Mat OpDetector::Joints2Dto3D(cv::Mat& Joints2D, cv::Mat& imD, double renderThres){
    cv::Mat Points3D = cv::Mat::zeros(1,25,CV_32FC3); // Only 3D positions of joints

    for(int i = 0; i < 25; i++){
        Vec3f joint2d, point3d;
        Vec2f point2d;
        double z = 0;
        joint2d = Joints2D.at<Vec3f>(i);
        if (joint2d[2] > renderThres){
            point2d[0] = joint2d[0];
            point2d[1] = joint2d[1];

            if (point2d[0]<imD.cols && point2d[1]<imD.rows){
                z = GetPointDepth(point2d, imD, 4)/1000;

                point3d[0] = (point2d[0] - mCam_ppx)*z/mCam_fx;
                point3d[1] = (point2d[1] - mCam_ppy)*z/mCam_fy;
                point3d[2] = z;
                Points3D.at<Vec3f>(i) = point3d;
            }
        }
    }
    return Points3D;
}

void OpDetector::Joints2DSeg(cv::Mat& imRGB, cv::Mat& Joints2D, cv::Mat& outputIm){
    /*
     * Joints2DSeg: calculate the segmentation part of the human body
     * according to the position of the 2D joints calculated from the openpose
     * Version 1.0: Only consider the joints located on torsos (1 2 5 8-14)
     * Input:
     *     - imRGB: color image (gray-scale in this project)
     *     - Joints2D: detected 2D joints
     *     - outputIm: output image
     */

    const int Width = imRGB.cols;
    const int Height = imRGB.rows;
    const double alpha = 0.4;
    const double scoreThres = 0.1;

    // Region {0,1,2,5,15,16,17,18}
    {
        int y_lb = Height - 1, y_ub = 0;
        int x_lb = Width - 1, x_ub = 0;
        const int num_joint = 8;
        int segJoints[num_joint]={0,1,2,5,15,16,17,18};
        double rowValue[num_joint], colValue[num_joint], score[num_joint];
        for (int i = 0; i < num_joint; i++){
            score[i] = Joints2D.at<Vec3f>(segJoints[i])[2];
            rowValue[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            if (score[i] > scoreThres && rowValue[i] < y_lb)
                y_lb = (int)rowValue[i];
            if (score[i] > scoreThres && rowValue[i] > y_ub)
                y_ub = (int)rowValue[i];
            colValue[i] = Joints2D.at<Vec3f>(segJoints[i])[0];
            if (score[i] > scoreThres && colValue[i] < x_lb)
                x_lb = (int)colValue[i];
            if (score[i] > scoreThres && colValue[i] > x_ub)
                x_ub = (int)colValue[i];
        }

        if (x_lb < x_ub && y_lb < y_ub){
            x_lb = x_lb < 0 ? 0 : x_lb;            y_lb = y_lb < 0 ? 0 : y_lb;
            x_ub = x_ub < 0 ? 1 : x_ub;            y_ub = y_ub < 0 ? 1 : y_ub;
            x_ub = x_ub > Width - 1 ? Width - 1 : x_ub;            y_ub = y_ub > Height - 1 ? Height - 1 : y_ub;
            x_lb = x_lb > Width - 1 ? Width - 2 : x_lb;            y_lb = y_lb > Height - 1 ? Height - 2 : y_lb;

            cv::Mat overlay = imRGB.clone();
            cv::rectangle(overlay, Point(x_lb,y_lb), Point(x_ub,y_ub), Scalar(0,0,255), -1);
            //cv::addWeighted(overlay, alpha, outputIm, 1 - alpha,  0, outputIm);
        }
    }

    // Region {1,2,3,5,6,8,9,12}
    {
        int y_lb = Height - 1, y_ub = 0;
        int x_lb = Width - 1, x_ub = 0;
        const int num_joint = 8;
        int segJoints[num_joint]={1,2,3,5,6,8,9,12};
        double rowValue[num_joint], colValue[num_joint], score[num_joint];
        for (int i = 0; i < num_joint; i++){
            score[i] = Joints2D.at<Vec3f>(segJoints[i])[2];
            rowValue[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            if (score[i] > scoreThres && rowValue[i] < y_lb)
                y_lb = (int)rowValue[i];
            if (score[i] > scoreThres && rowValue[i] > y_ub)
                y_ub = (int)rowValue[i];
            colValue[i] = Joints2D.at<Vec3f>(segJoints[i])[0];
            if (score[i] > scoreThres && colValue[i] < x_lb)
                x_lb = (int)colValue[i];
            if (score[i] > scoreThres && colValue[i] > x_ub)
                x_ub = (int)colValue[i];
        }
        if (x_lb < x_ub && y_lb < y_ub){
            x_lb = x_lb < 0 ? 0 : x_lb;            y_lb = y_lb < 0 ? 0 : y_lb;
            x_ub = x_ub < 0 ? 1 : x_ub;            y_ub = y_ub < 0 ? 1 : y_ub;
            x_ub = x_ub > Width - 1 ? Width - 1 : x_ub;            y_ub = y_ub > Height - 1 ? Height - 1 : y_ub;
            x_lb = x_lb > Width - 1 ? Width - 2 : x_lb;            y_lb = y_lb > Height - 1 ? Height - 2 : y_lb;

            cv::Mat overlay = outputIm.clone();
            cv::rectangle(overlay, Point(x_lb,y_lb), Point(x_ub,y_ub), Scalar(0,255,0), -1);
            cv::addWeighted(overlay, alpha, outputIm, 1 - alpha,  0, outputIm);
        }
    }

    // Region {8,9,10,12,13}
    {
        int y_lb = Height - 1, y_ub = 0;
        int x_lb = Width - 1, x_ub = 0;
        const int num_joint = 5;
        int segJoints[num_joint]={8,9,10,12,13};
        double rowValue[num_joint], colValue[num_joint], score[num_joint];
        for (int i = 0; i < num_joint; i++){
            score[i] = Joints2D.at<Vec3f>(segJoints[i])[2];
            rowValue[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            if (score[i] > scoreThres && rowValue[i] < y_lb)
                y_lb = (int)rowValue[i];
            if (score[i] > scoreThres && rowValue[i] > y_ub)
                y_ub = (int)rowValue[i];
            colValue[i] = Joints2D.at<Vec3f>(segJoints[i])[0];
            if (score[i] > scoreThres && colValue[i] < x_lb)
                x_lb = (int)colValue[i];
            if (score[i] > scoreThres && colValue[i] > x_ub)
                x_ub = (int)colValue[i];
        }

        if (x_lb < x_ub && y_lb < y_ub){
            x_lb = x_lb < 0 ? 0 : x_lb;            y_lb = y_lb < 0 ? 0 : y_lb;
            x_ub = x_ub < 0 ? 1 : x_ub;            y_ub = y_ub < 0 ? 1 : y_ub;

            x_ub = x_ub > Width - 1 ? Width - 1 : x_ub;            y_ub = y_ub > Height - 1 ? Height - 1 : y_ub;
            x_lb = x_lb > Width - 1 ? Width - 2 : x_lb;            y_lb = y_lb > Height - 1 ? Height - 2 : y_lb;

            cv::Mat overlay = outputIm.clone();
            cv::rectangle(overlay, Point(x_lb,y_lb), Point(x_ub,y_ub), Scalar(255,0,255), -1);
            cv::addWeighted(overlay, alpha, outputIm, 1 - alpha,  0, outputIm);
        }
    }

    // Region {10,11,22,23,24};
    {
        int y_lb = Height - 1, y_ub = 0;
        int x_lb = Width - 1, x_ub = 0;
        const int num_joint = 5;
        int segJoints[num_joint]={10,11,22,23,24};
        double rowValue[num_joint], colValue[num_joint], score[num_joint];
        for (int i = 0; i < num_joint; i++){
            score[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            rowValue[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            if (score[i] > scoreThres && rowValue[i] < y_lb)
                y_lb = (int)rowValue[i];
            if (score[i] > scoreThres && rowValue[i] > y_ub)
                y_ub = (int)rowValue[i];
            colValue[i] = Joints2D.at<Vec3f>(segJoints[i])[0];
            if (score[i] > scoreThres && colValue[i] < x_lb)
                x_lb = (int)colValue[i];
            if (score[i] > scoreThres && colValue[i] > x_ub)
                x_ub = (int)colValue[i];
        }

        if (x_lb < x_ub && y_lb < y_ub){
            x_lb = x_lb < 0 ? 0 : x_lb;            y_lb = y_lb < 0 ? 0 : y_lb;
            x_ub = x_ub < 0 ? 1 : x_ub;            y_ub = y_ub < 0 ? 1 : y_ub;
            x_ub = x_ub > Width - 1 ? Width - 1 : x_ub;            y_ub = y_ub > Height - 1 ? Height - 1 : y_ub;
            x_lb = x_lb > Width - 1 ? Width - 2 : x_lb;            y_lb = y_lb > Height - 1 ? Height - 2 : y_lb;

            cv::Mat overlay = outputIm.clone();
            cv::rectangle(overlay, Point(x_lb,y_lb), Point(x_ub,y_ub), Scalar(255,255,0), -1);
            cv::addWeighted(overlay, alpha, outputIm, 1 - alpha,  0, outputIm);
        }
    }

    // Region {13,14,19,20,21};
    {
        int y_lb = Height - 1, y_ub = 0;
        int x_lb = Width - 1, x_ub = 0;
        const int num_joint = 5;
        int segJoints[num_joint]={13,14,19,20,21};
        double rowValue[num_joint], colValue[num_joint], score[num_joint];
        for (int i = 0; i < num_joint; i++){
            score[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            rowValue[i] = Joints2D.at<Vec3f>(segJoints[i])[1];
            if (score[i] > scoreThres && rowValue[i] < y_lb)
                y_lb = (int)rowValue[i];
            if (score[i] > scoreThres && rowValue[i] > y_ub)
                y_ub = (int)rowValue[i];
            colValue[i] = Joints2D.at<Vec3f>(segJoints[i])[0];
            if (score[i] > scoreThres && colValue[i] < x_lb)
                x_lb = (int)colValue[i];
            if (score[i] > scoreThres && colValue[i] > x_ub)
                x_ub = (int)colValue[i];
        }

        if (x_lb < x_ub && y_lb < y_ub){
            x_lb = x_lb < 0 ? 0 : x_lb;            y_lb = y_lb < 0 ? 0 : y_lb;
            x_ub = x_ub < 0 ? 1 : x_ub;            y_ub = y_ub < 0 ? 1 : y_ub;
            x_ub = x_ub > Width - 1 ? Width - 1 : x_ub;            y_ub = y_ub > Height - 1 ? Height - 1 : y_ub;
            x_lb = x_lb > Width - 1 ? Width - 2 : x_lb;            y_lb = y_lb > Height - 1 ? Height - 2 : y_lb;

            cv::Mat overlay = outputIm.clone();
            cv::rectangle(overlay, Point(x_lb,y_lb), Point(x_ub,y_ub), Scalar(255,255,0), -1);
            cv::addWeighted(overlay, alpha, outputIm, 1 - alpha,  0, outputIm);
        }
    }

}

void OpDetector::Joints3DSeg(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat& Joints3D, cv::Mat& outputIm){
    /*
     * Joints3DSeg: calculate the segmentation part of the human body
     * according to the position of the 2D & 3D joints calculated from the openpose
     * Version 1.0: Segments is determined according to the joints detected by OP
     * Input:
     *     - imRGB: color image (gray-scale in this project)
     *     - imD: depth image
     *     - Joints2D: detected 2D joints
     *     - Joints3D: calculated 3D joints
     *     - outputIm: output image (MASK - CV_8UC1)
     */

    // Pixel value of the point that is determined as a part of human body (dynamic)
    cv::Scalar maskValue = cv::Scalar(0);

                                //2-3   3-4   5-6   6-7  1 -2  1-5  0-1  1-8  2-12 5-9  8-9  8-12 9-10  12-13 10-11 13-14
    const float radiusLink[28] = {0.08, 0.05, 0.08, 0.05, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1, 0.1, 0.12, 0.12, 0.1,  0.1,
                                  0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
                                //11-22 11-23 11-24 22-23 22-24 23-24 14-19 14-20 14-21 19-20 19-21 20-21
    const float lengthLink[28] = {0.25, 0.25, 0.25, 0.25, 0.2, 0.2, 0.2, 0.5, 0.4, 0.4, 0.15, 0.15, 0.4, 0.4, 0.3,  0.3,
                                  0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    //11-22 11-23 11-24 22-23 22-24 23-24 14-19 14-20 14-21 19-20 19-21 20-21
    const int joint1[28] = {2, 3, 5, 6, 1, 1, 0, 1, 2, 5, 8, 8, 9, 12, 10, 13, 11, 11, 11, 22, 22, 23, 14, 14, 14, 19, 19, 20};
    const int joint2[28] = {3, 4, 6, 7, 2, 5, 1, 8, 12,9, 9, 12,10,13, 11, 14, 22, 23, 24, 23, 24, 24, 19, 20, 21, 20, 21, 21};

    float Z_max = 0;
    for (int i = 0; i < 25; i++){
        Z_max = Joints3D.at<cv::Vec3f>(i)[2] > Z_max ? Joints3D.at<cv::Vec3f>(i)[2] : Z_max;
    }
    if (Z_max == 0) Z_max = mZ_max_last;
    mZ_max_last = Z_max;

    // 1. Head
    {
        float p_z = Joints3D.at<cv::Vec3f>(0)[2];
        int p_x = (int)Joints2D.at<cv::Vec3f>(0)[0];
        int p_y = (int)Joints2D.at<cv::Vec3f>(0)[1];
        int rHead_px;
        if (p_z == 0)
            rHead_px = (int)((0.15/(Z_max))*mCam_fx);
        else
            rHead_px = (int)((0.15/(p_z))*mCam_fx);

        cv::circle(outputIm, Point(p_x,p_y), rHead_px, maskValue, -1);
    }

    // OTHERS
    for (int i = 0; i < 28; i++)
    {
        int j1 = joint1[i]; int j2 = joint2[i];
        int p_x1 = (int)Joints2D.at<cv::Vec3f>(j1)[0];
        int p_y1 = (int)Joints2D.at<cv::Vec3f>(j1)[1];
        float p_z1 = Joints3D.at<cv::Vec3f>(j1)[2];
        int p_x2 = (int)Joints2D.at<cv::Vec3f>(j2)[0];
        int p_y2 = (int)Joints2D.at<cv::Vec3f>(j2)[1];
        float p_z2 = Joints3D.at<cv::Vec3f>(j2)[2];
        int rAxisX_px, rAxisY_px;
        if (p_z1 < 0) p_z1 = 0;
        if (p_z2 < 0) p_z2 = 0;

        float rElp = radiusLink[i]; float lElp = lengthLink[i];
        if (p_z1 > 0 || p_z2 > 0) { // One of the point has depth value
            if (p_z1 > 0 && p_z2 == 0) {
                rAxisX_px = (int)((rElp/p_z1)*mCam_fx); rAxisY_px = (int)((lElp/p_z1)*mCam_fx); }
            else if (p_z1 == 0 && p_z2 > 0){
                rAxisX_px = (int)((rElp/p_z2)*mCam_fx); rAxisY_px = (int)((lElp/p_z2)*mCam_fx); }
            else{
                rAxisX_px = (int)((2*rElp/(p_z1+p_z2))*mCam_fx); rAxisY_px = (int)((2*lElp/((p_z1+p_z2)))*mCam_fx);  }
        }
        else{
            rAxisX_px = rElp/Z_max*mCam_fx;  rAxisY_px = lElp/Z_max*mCam_fx;   }

        double angles;
        if (p_x2 == p_x1 )
            angles = 0.0;
        else
            angles = -3.1415/2.0 + atan((p_y2-p_y1)/(p_x2-p_x1));
        //cout << "Link: " << j1 << "-" << j2 << ": " << rAxisX_px << " " << rAxisY_px << " pz1: " << p_z1 << " p_z2: " << p_z2 << endl;
        cv::ellipse(outputIm, Point((p_x1+p_x2)/2, (p_y1+p_y2)/2), Size(rAxisX_px, rAxisY_px), angles*180/3.1415, 0.0, 360.0, maskValue, -1);
    }
}

float OpDetector::GetPointDepth(cv::Vec2f point2D, cv::Mat& imD, int depth_radius){
    /*
     * GetPointDepth: get the depth value of a given pixel. Due to that the depth image
     * typically contains several zero values. Hence, we need to estimate the depth value
     * according to its neighbors
     * Input:
     *     - point2D (x,y)
     *     - imD: depth image
     *     - depth_radius: radius of the neighborhood
     */
    int z = 0, z_valid = 0, z_cnt = 0;
    int x = point2D[0];
    int y = point2D[1];
    int x_lb = x-depth_radius, x_ub = x+depth_radius;
    int y_lb = y-depth_radius, y_ub = y+depth_radius;

    // Bound limitation
    x_lb = x_lb<0 ? 0:x_lb;        x_lb = x_lb>imD.cols-1 ? imD.cols-1:x_lb;
    x_ub = x_ub<=x_lb ? x_lb+1:x_ub;  x_ub = x_ub>imD.cols-1 ? imD.cols-1:x_ub;
    y_lb = y_lb<0 ? 0:y_lb;        y_lb = y_lb>imD.rows-1 ? imD.rows-1:y_lb;
    y_ub = y_ub<=y_lb ? y_lb+1:y_ub;  y_ub = y_ub>imD.rows-1 ? imD.rows-1:y_ub;

    cv::Mat z_ROI = imD.rowRange(y_lb,y_ub).colRange(x_lb,x_ub);

    // Get valid depth value from non-zero elements in a ROI
    z_ROI.convertTo(z_ROI, CV_32FC1);
    std::vector<float> unik_out = unique(z_ROI, true);
    if (unik_out.size()>0)
        if (unik_out[0] == 0)
            unik_out.pop_back();

    for (unsigned int i = 0; i < unik_out.size(); i++){
        if (z_valid==0)
            z_valid = unik_out[i];
        if (z_valid>0)
            if (unik_out[i] > z_valid + 10){
                break;
            }
        z = z + unik_out[i];
        if (unik_out[i]>0)
            z_cnt++;
    }
    z = z / (z_cnt + 1e-23);
    return z;
}

void OpDetector::Plot2DJoints(cv::Mat Joints2D, cv::Mat& im, double renderThres){
    int links[2][20] = {{0, 2, 2, 4, 5, 5, 7, 8 ,8 ,8,10,10,22,23,24,13,13,19,20,21},
                        {1, 1, 3, 3, 1, 6, 6, 1, 9,12, 9,11,11,11,11,12,14,14,14,14}};
    for(int i=0; i<20; i++){
        Vec3f jStart, jEnd;
        jStart = Joints2D.at<Vec3f>(links[0][i]);
        jEnd = Joints2D.at<Vec3f>(links[1][i]);
        Point jStartPlot, jEndPlot;
        float scoreStart, scoreEnd;
        jStartPlot.x = jStart[0];
        jStartPlot.y = jStart[1];
        scoreStart = jStart[2];
        jEndPlot.x = jEnd[0];
        jEndPlot.y = jEnd[1];
        scoreEnd = jEnd[2];
        if (scoreStart > renderThres && scoreEnd > renderThres){
            //void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
            //void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0);
            cv::circle(im, jStartPlot, 3, Scalar(255,0,0), 4);
            cv::circle(im, jEndPlot, 3, Scalar(255,0,0), 4);
            cv::line(im, jStartPlot, jEndPlot, Scalar(255-i*10, i*10, 255-i*10), 5);
        }
    }
}

std::vector<float> OpDetector::unique(const cv::Mat& input, bool sortflag = false){
    if (input.channels() > 1 || input.type() != CV_32F)
    {
        std::cerr << "unique !!! Only works with CV_32F 1-channel Mat" << std::endl;
        return std::vector<float>();
    }

    std::vector<float> out;
    for (int y = 0; y < input.rows; ++y)
    {
        const float* row_ptr = input.ptr<float>(y);
        for (int x = 0; x < input.cols; ++x)
        {
            float value = row_ptr[x];

            if ( std::find(out.begin(), out.end(), value) == out.end() )
                out.push_back(value);
        }
    }

    if (sortflag)
        std::sort(out.begin(), out.end());

    return out;
}

cv::Mat OpDetector::GetInformPersonJoint(cv::Mat Joints2D, double renderThres, cv::Size Im_size){
    cv::Mat InformPerson(1, 25, CV_32FC3, cv::Mat::AUTO_STEP);
    int N = Joints2D.rows;
    double Thres = renderThres;
    double score_avg;
    vector<Mat> channels(3);
    split(Joints2D, channels);
    for (int i = 0; i < N; i++){
        // 1: Combine joints that stored in different people
        if (N > 1){
            for (int j = 0; j < N; j++){
                if (i != j){
                    vector<Mat> channels1(3), channels2(3);
                    split(Joints2D.row(i), channels1);
                    split(Joints2D.row(j), channels2);
                    cv::Mat idx1, idx2, idx_unit;
                    threshold(channels1[2], idx1, 1e-23, 1, CV_THRESH_BINARY);
                    threshold(channels2[2], idx2, 1e-23, 1, CV_THRESH_BINARY);
                    Scalar cnt1 = sum(idx1);
                    Scalar cnt2 = sum(idx2);
                    idx_unit = idx1 + idx2;
                    threshold(idx_unit, idx_unit, 1e-23, 1, CV_THRESH_BINARY);
                    Scalar cnt_unit = sum(idx_unit);
                    if (cnt1[0] + cnt2[0] == cnt_unit[0]){
                        Scalar score_sum = sum(channels1[2] + channels2[2]);
                        score_avg = score_sum[0]/(cnt_unit[0] + 1e-23);
                        if (score_avg > Thres){
                            Thres = score_avg;
                            cv::Mat CombinedJoint = Joints2D.row(i) + Joints2D.row(j);
                            CombinedJoint.copyTo(InformPerson);
                        }
                    }
                    else{
                        Scalar score_sum = sum(channels1[2]);
                        score_avg = score_sum[0]/(cnt1[0] + 1e-23);
                        if (score_avg > Thres){
                            Thres = score_avg;
                            cv::Mat CombinedJoint = Joints2D.row(i) + Joints2D.row(j);
                            Joints2D.row(i).copyTo(InformPerson);
                        }
                    }

                }
            }
        }
        else{
            Joints2D.copyTo(InformPerson);
        }
    }

    return InformPerson;
}

cv::KalmanFilter OpDetector::KFInitialization(const int stateNum, const int measureNum, double wk, double vk, double pk){
    KalmanFilter KF(stateNum, measureNum, 0);
    Mat state(stateNum, 1, CV_32FC1); // STATE (x, y, dx, dy)
    Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F); //measurement(x,y)

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, Scalar::all(wk));
    cv::setIdentity(KF.measurementNoiseCov, Scalar::all(vk));
    cv::setIdentity(KF.errorCovPost, Scalar::all(pk));

    Mat transitionMatrix = cv::Mat::eye(stateNum, stateNum, CV_32FC1);

    for (int i = 0; i < measureNum; i++){
        transitionMatrix.at<float>(i, measureNum+i) = 0.5;
    }
    KF.transitionMatrix = transitionMatrix;

    return KF;
}

cv::Mat OpDetector::KFupdate(cv::Mat Joints3D, const int stateNum, const int measureNum){
    int LowerLimb[13]={8,9,10,11,12,13,14,19,20,21,22,23,24};
    cv::Mat Joints3D_KFsmooth;
    Joints3D_KFsmooth = Joints3D.clone();

    /// KALMAN SMOOTHER
    for (int i = 0; i < 13; i ++){
        int idx = LowerLimb[i];
        Vec3f jointRaw = Joints3D_KFsmooth.at<cv::Vec3f>(idx);
        Vec3f jointSmooth;
        Mat prediction = KFs3D[idx].predict();
        Mat measurementPt = Mat::zeros(measureNum, 1, CV_32F); //measurement(x,y)
        if (jointRaw[2] > 0){  // If there is measurement
            measurementPt.at<float>(0) = jointRaw[0];
            measurementPt.at<float>(1) = jointRaw[1];
            measurementPt.at<float>(2) = jointRaw[2];
        }
        else{ // If there is  no measurement
            cv::Mat lastState = KFs3D[idx].statePost;
            measurementPt = KFs3D[idx].measurementMatrix*lastState;
        }

        Mat estimatedPt = KFs3D[idx].correct(measurementPt);
        KFs3D[idx].statePost = KFs3D[idx].statePre + KFs3D[idx].gain * KFs3D[idx].temp5;
            // Refinement according to the human body constraints

        Mat updatePt;
        if (idx == HIP_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(HIP_C), mHumanParams.Link_hip_L);
        else if (idx == HIP_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(HIP_C), mHumanParams.Link_hip_R);
        else if (idx == KNEE_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(HIP_R), mHumanParams.Link_thigh_R);
        else if (idx == KNEE_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(HIP_L), mHumanParams.Link_thigh_L);
        else if (idx == ANKLE_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(KNEE_R), mHumanParams.Link_shank_R);
        else if (idx == ANKLE_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(KNEE_L), mHumanParams.Link_shank_L);
        else if (idx == TOE_IN_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_foot_R);
        else if (idx == TOE_OUT_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_foot_R);
        else if (idx == HEEL_R)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_heel_R);
        else if (idx == TOE_IN_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_foot_L);
        else if (idx == TOE_OUT_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_foot_L);
        else if (idx == HEEL_L)
            updatePt = updateMeasurement(estimatedPt.rowRange(0,3), Joints3D.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_heel_L);

        if (i>0){
            KFs3D[idx].statePost.at<float>(0) = updatePt.at<float>(0);
            KFs3D[idx].statePost.at<float>(1) = updatePt.at<float>(1);
            KFs3D[idx].statePost.at<float>(2) = updatePt.at<float>(2);
            jointSmooth[0] = updatePt.at<float>(0);
            jointSmooth[1] = updatePt.at<float>(1);
            jointSmooth[2] = updatePt.at<float>(2);
            Joints3D_KFsmooth.at<cv::Vec3f>(idx) = jointSmooth;
        }
        else{
            jointSmooth[0] = estimatedPt.at<float>(0);
            jointSmooth[1] = estimatedPt.at<float>(1);
            jointSmooth[2] = estimatedPt.at<float>(2);
            Joints3D_KFsmooth.at<cv::Vec3f>(idx) = jointSmooth;
        }
    }
    return Joints3D_KFsmooth;
}

float OpDetector::CalcSkelDist(cv::Mat skel_curr, cv::Mat skel_last, int *JointSet, int JointSize){
    /*
     * skel_curr: 3D skeleton in current frame: J = [25*1*3] (cv::Vec3f)
     * skel_last: 3D skeleton in last frame: J = [25*1*3] (cv::Vec3f)
     * JointSet: selected joints needed to calculate the distance
     */
    float SkelDist = 0.0;

    for (int i = 0; i < JointSize; i++){
        int idx = JointSet[i];
        // Only calculate the distance between two valid joints
        if (skel_curr.at<cv::Vec3f>(idx)[2]>0 && skel_last.at<cv::Vec3f>(idx)[2] >0){
            SkelDist = SkelDist + cv::norm(skel_curr.at<cv::Vec3f>(idx), skel_last.at<cv::Vec3f>(idx));
        }

    }
    return SkelDist;
}

void OpDetector::InitHumanParams(struct HumanParams *mHumanParams){
    mHumanParams->Link_thigh_L = 0.0;
    mHumanParams->Link_thigh_R = 0.0;
    mHumanParams->Link_shank_L = 0.0;
    mHumanParams->Link_shank_R = 0.0;
    mHumanParams->Link_foot_L = 0.0;
    mHumanParams->Link_foot_R = 0.0;
    mHumanParams->Link_hip_L = 0.0;
    mHumanParams->Link_hip_R = 0.0;
    mHumanParams->Link_heel_L = 0.0;
    mHumanParams->Link_heel_R = 0.0;
    mHumanParams->Cnt_thigh_L = 0.0;
    mHumanParams->Cnt_thigh_R = 0.0;
    mHumanParams->Cnt_shank_L = 0.0;
    mHumanParams->Cnt_shank_R = 0.0;
    mHumanParams->Cnt_foot_L = 0.0;
    mHumanParams->Cnt_foot_R = 0.0;
    mHumanParams->Cnt_hip_L = 0.0;
    mHumanParams->Cnt_hip_R = 0.0;
    mHumanParams->Cnt_heel_L = 0.0;
    mHumanParams->Cnt_heel_R = 0.0;
}

void OpDetector::UpdateHumanParams(cv::Mat Joints3D, struct HumanParams *mHumanParams){
    int LowerPair[2][12]={{8, 8, 9,10,11,11,11,12,13,14,14,14},
                          {9,12,10,11,22,23,24,13,14,19,20,21}};
    for (int i = 0; i < 12; i ++){
        cv::Vec3f P1 = Joints3D.at<cv::Vec3f>(LowerPair[0][i]);
        cv::Vec3f P2 = Joints3D.at<cv::Vec3f>(LowerPair[1][i]);
        double LinkLength = 0.0;
        switch (i){
            case 0: // HIP_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_hip_R++;
                }
                if (mHumanParams->Cnt_hip_R == 0 || mHumanParams->Cnt_hip_R == 1)
                    mHumanParams->Link_hip_R = LinkLength;
                else
                    mHumanParams->Link_hip_R = LinkLength*(double)(1.0/mHumanParams->Cnt_hip_R) +
                                              mHumanParams->Link_hip_R*(double)((mHumanParams->Cnt_hip_R-1.0)/mHumanParams->Cnt_hip_R);
            case 1: // HIP_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_hip_L++;
                }
                if (mHumanParams->Cnt_hip_L == 0 || mHumanParams->Cnt_hip_L == 1)
                    mHumanParams->Link_hip_L = LinkLength;
                else
                    mHumanParams->Link_hip_L = LinkLength*(1.0/mHumanParams->Cnt_hip_L) +
                                              mHumanParams->Link_hip_L*((mHumanParams->Cnt_hip_L-1.0)/mHumanParams->Cnt_hip_L);
            case 2: // THIGH_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_thigh_R++;
                }
                if (mHumanParams->Cnt_thigh_R == 0 || mHumanParams->Cnt_thigh_R == 1)
                    mHumanParams->Link_thigh_R= LinkLength;
                else
                    mHumanParams->Link_thigh_R = LinkLength*(1.0/mHumanParams->Cnt_thigh_R) +
                                                mHumanParams->Link_thigh_R*((mHumanParams->Cnt_thigh_R-1.0)/mHumanParams->Cnt_thigh_R);
            case 3: // SHANK_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_shank_R++;
                }
                if (mHumanParams->Cnt_shank_R == 0 || mHumanParams->Cnt_shank_R == 1)
                    mHumanParams->Link_shank_R= LinkLength;
                else
                    mHumanParams->Link_shank_R = LinkLength*(1.0/mHumanParams->Cnt_shank_R) +
                                                 mHumanParams->Link_shank_R*((mHumanParams->Cnt_shank_R-1.0)/mHumanParams->Cnt_shank_R);
            case 4: // FOOT_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_foot_R++;
                }
                if (mHumanParams->Cnt_foot_R == 0 || mHumanParams->Cnt_foot_R == 1)
                    mHumanParams->Link_foot_R= LinkLength;
                else
                    mHumanParams->Link_foot_R = LinkLength*(1.0/mHumanParams->Cnt_foot_R) +
                                               mHumanParams->Link_foot_R*((mHumanParams->Cnt_foot_R-1.0)/mHumanParams->Cnt_foot_R);
            case 5: // FOOT_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_foot_R++;
                }
                if (mHumanParams->Cnt_foot_R == 0 || mHumanParams->Cnt_foot_R == 1)
                    mHumanParams->Link_foot_R= LinkLength;
                else
                    mHumanParams->Link_foot_R = LinkLength*(1.0/mHumanParams->Cnt_foot_R) +
                                                mHumanParams->Link_foot_R*((mHumanParams->Cnt_foot_R-1.0)/mHumanParams->Cnt_foot_R);
            case 6: // HEEL_R
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_heel_R++;
                }
                if (mHumanParams->Cnt_heel_R == 0 || mHumanParams->Cnt_heel_R == 1)
                    mHumanParams->Link_heel_R= LinkLength;
                else
                    mHumanParams->Link_heel_R = LinkLength*(1.0/mHumanParams->Cnt_heel_R) +
                                                mHumanParams->Link_heel_R*((mHumanParams->Cnt_heel_R-1.0)/mHumanParams->Cnt_heel_R);
            case 7: //THIGH_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_thigh_L++;
                }
                if (mHumanParams->Cnt_thigh_L == 0 || mHumanParams->Cnt_thigh_L == 1)
                    mHumanParams->Link_thigh_L= LinkLength;
                else
                    mHumanParams->Link_thigh_L = LinkLength*(1.0/mHumanParams->Cnt_thigh_L) +
                                                 mHumanParams->Link_thigh_L*((mHumanParams->Cnt_thigh_L-1.0)/mHumanParams->Cnt_thigh_L);
            case 8: //SHANK_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_shank_L++;
                }
                if (mHumanParams->Cnt_shank_L == 0 || mHumanParams->Cnt_shank_L == 1)
                    mHumanParams->Link_shank_L= LinkLength;
                else
                    mHumanParams->Link_shank_L = LinkLength*(1.0/mHumanParams->Cnt_shank_L) +
                                                 mHumanParams->Link_shank_L*((mHumanParams->Cnt_shank_L-1.0)/mHumanParams->Cnt_shank_L);
            case 9: // FOOT_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_foot_L++;
                }
                if (mHumanParams->Cnt_foot_L == 0 || mHumanParams->Cnt_foot_L == 1)
                    mHumanParams->Link_foot_L= LinkLength;
                else
                    mHumanParams->Link_foot_L = LinkLength*(1.0/mHumanParams->Cnt_foot_L) +
                                               mHumanParams->Link_foot_L*((mHumanParams->Cnt_foot_L-1.0)/mHumanParams->Cnt_foot_L);
            case 10: // FOOT_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_foot_L++;
                }
                if (mHumanParams->Cnt_foot_L == 0 || mHumanParams->Cnt_foot_L == 1)
                    mHumanParams->Link_foot_L= LinkLength;
                else
                    mHumanParams->Link_foot_L = LinkLength*(1.0/mHumanParams->Cnt_foot_L) +
                                                mHumanParams->Link_foot_L*((mHumanParams->Cnt_foot_L-1.0)/mHumanParams->Cnt_foot_L);
            case 11: // HEEL_L
                if (P1[2] > 0 && P2[2]){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    mHumanParams->Cnt_heel_L++;
                }
                if (mHumanParams->Cnt_heel_L == 0 || mHumanParams->Cnt_heel_L == 1)
                    mHumanParams->Link_heel_L= LinkLength;
                else
                    mHumanParams->Link_foot_L = LinkLength*(1.0/mHumanParams->Cnt_heel_L) +
                                                mHumanParams->Link_heel_L*((mHumanParams->Cnt_heel_L-1.0)/mHumanParams->Cnt_heel_L);
        }
    }
}

cv::Mat OpDetector::updateMeasurement(cv::Mat measurementPt, cv::Vec3f rootPt, double linkConstraint ){
    Mat updatePt;
    updatePt = measurementPt.clone();
    if (linkConstraint > 0){
        Mat rootPtmat = Mat::zeros(3,1,CV_32F);
        rootPtmat.at<float>(0) = rootPt[0];
        rootPtmat.at<float>(1) = rootPt[1];
        rootPtmat.at<float>(2) = rootPt[2];
        double vec_x = updatePt.at<float>(0) - rootPtmat.at<float>(0);
        double vec_y = updatePt.at<float>(1) - rootPtmat.at<float>(1);
        double vec_z = updatePt.at<float>(2) - rootPtmat.at<float>(2);
        double linkLength = cv::norm(updatePt, rootPtmat, cv::NORM_L2);
        if (linkLength > linkConstraint){
            updatePt.at<float>(0) = rootPtmat.at<float>(0) + sqrt(linkConstraint/linkLength)*vec_x;
            updatePt.at<float>(1) = rootPtmat.at<float>(1) + sqrt(linkConstraint/linkLength)*vec_y;
            updatePt.at<float>(2) = rootPtmat.at<float>(2) + sqrt(linkConstraint/linkLength)*vec_z;
            //out << "Data: " << linkLength << " " << linkConstraint << " " << measurementPt << " " << updatePt << " " << rootPtmat << endl;
        }
    }
    return updatePt;
}

void OpDetector::RequestFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool OpDetector::CheckFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void OpDetector::SetFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool OpDetector::isFinished(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void OpDetector::RequestStop(){
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool OpDetector::isStopped(){
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool OpDetector::Stop(){
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

void OpDetector::Release(){
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

} // ORB_SLAM2