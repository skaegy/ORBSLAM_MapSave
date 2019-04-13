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
    /* OpDetector:
     * Initialization of the OP detector, for detecting & tracking & smoothing the 2D skeleton
     * Generate 3D skeleton according to RGB-D images
     * Smoothing 3D skeleton using the KF + Link constraints + Angle Constraints
     * -- Input:
     *      strOpenposeSettingsFile: the path to openpose setting file
     *      bHumanPose: the flag of detecting human pose (true & false)
     *      SensorMode: Monocular, Sterep, & RGB-D
     * */
    mbHumanPose = bHumanPose;
    cv::FileStorage fs(strOpenposeSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Unable to open openpose parameter file!" << endl;
    }

    // ----- FLAGS for openpose ----- //
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

    // ----- Parameters for Camera & Kalman Filter ----- //
    mCamParas.fx = fs["Camera.fx"];
    mCamParas.fy = fs["Camera.fy"];
    mCamParas.ppx = fs["Camera.cx"];
    mCamParas.ppy = fs["Camera.cy"];
    mCamParas.width = fs["Camera.width"];
    mCamParas.height = fs["Camera.height"];
    mKFparameters.wk = fs["KF.wk"];
    mKFparameters.vk = fs["KF.vk"];
    mKFparameters.pk = fs["KF.pk"];
    mKFparameters.stateNum = 6;
    mKFparameters.measureNum = 3;

    fs.release();
    mSensor = SensorMode;

    mNormFloor[0] = 0;
    mNormFloor[1] = -1;
    mNormFloor[2] = 0;

    /* ------------------ OP-Tracking ------------------ */
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

    // -------- 3D KALMAN FILTER INITIALIZATION ------- //
    for (int i = 0; i < 25; i ++){
        KFs3D[i] =KFinit(&mKFparameters);
    }

    // -------- Human skeleton parameters initialization ------- //
    //TODO: Skeleton tree for smooth
    InitHumanParams();

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
const int stateNum = 6;
const int measureNum = 3;
while (!mbStopped ) {
    usleep(1e5);
    if (mbHumanPose && !mlLoadImage.empty()) {
        // Load images for extracting human pose
        mMutexColorIm.lock();
        cv::Mat inputImage = mlLoadImage.front();
        /// -------- pop
        mlLoadImage.pop_back();
        mMutexColorIm.unlock();

        // Openpose for single person tracking
        auto datumProcessed = opWrapper.emplaceAndPop(inputImage);
        cv::Mat joints2D;
        cv::Mat OutputImage;
        if (datumProcessed != nullptr)
        {
            const auto poseKeypoints = datumProcessed->at(0).poseKeypoints;
            OutputImage = datumProcessed->at(0).cvOutputData;
            joints2D = poseKeypoints.getConstCvMat();

            mMutexOutputIm.lock();
            mlRenderPoseImage.push_front(OutputImage);
            if (mlRenderPoseImage.size() > 1)
                mlRenderPoseImage.pop_back();
            mMutexOutputIm.unlock();
        }

        // OP-Pose from Image
        /*
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

        if (joints2D.cols > 0){

            /* OP-Pose from image
            cv::Mat keyJoint2D =GetInformPersonJoint(joints2D, render_threshold, cv::Size(inputImage.cols, inputImage.rows));
            auto newposeKeypoints = cvMatToOpOutput.createArray(keyJoint2D, scaleInputToOutput, outputResolution);
            poseRenderer.renderPose(newoutputArray, newposeKeypoints, scaleInputToOutput);
            auto newOutputImage = opOutputToCvMat.formatToCvMat(newoutputArray);
            */

            /* OP-Tracking */
            mMutexJoint.lock();
            mvOpJoints2D.push_back(joints2D.clone());
            mMutexJoint.unlock();

            //Plot2DJoints(joints2D, inputImage, render_threshold);

            if (!mlLoadDepth.empty()){
                //const auto timeBegin = std::chrono::high_resolution_clock::now();
                /// Get depth value and map to 3D skeleton
                mMutexDepthIm.lock();
                cv::Mat inputDepth = mlLoadDepth.front();
                mlLoadDepth.pop_back();
                mMutexDepthIm.unlock();

                mJoints3D = Skeleton2Dto3D(joints2D, inputDepth, mOPflag_render_threshold);
                // TODO: Depth value predict using KF? to estimate the depth of the joint without measured value
                if (!mvJoints3DEKF.empty())
                    mJoints3D_last = mvJoints3DEKF.back();
                else
                    mJoints3D_last = mJoints3D.clone();

                mMutexJoint.lock();
                mvJoints3Draw.push_back(mJoints3D);
                mMutexJoint.unlock();

                /// KALMAN SMOOTHER
                mJoints3D_EKFsmooth = KFupdate(mJoints3D, stateNum, measureNum);
                // TODO: LengthRefine
                //KFupdate(mJoints3D, mJoints3D_last, stateNum, measureNum);


                /// BODY PHYSICAL CONSTRAINTS
                // Update the human body parameters during the tracking,
                // which will be used as the follow physical constraints in the smoothing
                UpdateHumanParams_LinkLength(mJoints3D_EKFsmooth, 20);

                /// Generate the mask according to the joints captured from openpose
                mHumanMask = cv::Mat::ones(inputImage.size(),CV_8UC1);
                mHumanMask.setTo(cv::Scalar(255));

                //Skeleton3DSeg(inputDepth,joints2D,mJoints3D_EKFsmooth,mHumanMask);
                SkeletonSquareMask(inputDepth, joints2D, mHumanMask);

                mMutexOutputIm.lock();
                mlHumanMask.push_front(mHumanMask.clone());
                if (mlHumanMask.size() > 1)
                    mlHumanMask.pop_back();
                mMutexOutputIm.unlock();

                mMutexJoint.lock();
                mvJoints3DEKF.push_back(mJoints3D_EKFsmooth.clone());
                mvTimestamp.push_back(mlLoadTimestamp.front());

                /// --- pop
                mlLoadTimestamp.pop_back();
                mMutexJoint.unlock();

                //const auto now_time = std::chrono::high_resolution_clock::now();
                //const auto totalTimeMilSec =(double) std::chrono::duration_cast<std::chrono::nanoseconds>(now_time - timeBegin).count()
                        //* 1e-6;
                //cout << "Skeleton process time: " << totalTimeMilSec << " (ms)" << endl;
            }
        }
        else{
            mMutexOutputIm.lock();
            mlRenderPoseImage.push_front(inputImage);
            if (mlRenderPoseImage.size() > 1)
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

void OpDetector::OpLoadImageMonocular(const cv::Mat &im, const double timestamp){
    /* OpLoadImageMonocular
     * Load RGB image and time stamp for extracting human skeleton (2D & 3D)
     * - Input:
     *      - im[cv::Mat]: RGB image
     *      - timestamp[double]
     * - The input image is stored in a list for post-processing
     * */
    cv::Mat BufMat = im.clone();
    mMutexColorIm.lock();
    mlLoadImage.push_front(BufMat);
    if (mlLoadImage.size()>1)
        mlLoadImage.pop_back();
    mlLoadTimestamp.push_front(timestamp);
    if (mlLoadTimestamp.size()>1)
        mlLoadTimestamp.pop_back();
    mMutexColorIm.unlock();
}

void OpDetector::OpLoadImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double timestamp){
    /* OpLoadImageMonocular
     * Load RGB and Depth image and time stamp for extracting human skeleton (2D & 3D)
     * - Input:
     *      - imRGB[cv::Mat]: RGB image
     *      - imD[cv::Mat]: depth image
     *      - timestamp[double]
     * - The input images are stored in two lists for post-processing
     * */

    cv::Mat BufRGB = imRGB.clone();
    cv::Mat BufDepth = imD.clone();
    mMutexColorIm.lock();
    mlLoadImage.push_front(BufRGB);
    if (mlLoadImage.size()>1)
        mlLoadImage.pop_back();
    mMutexColorIm.unlock();
    mMutexDepthIm.lock();
    mlLoadDepth.push_front(BufDepth);
    if (mlLoadDepth.size()>1)
        mlLoadDepth.pop_back();
    mlLoadTimestamp.push_front(timestamp);
    if (mlLoadTimestamp.size()>1)
        mlLoadTimestamp.pop_back();
    mMutexDepthIm.unlock();
}

cv::Vec3f OpDetector::rs_pixel2point(cv::Point2i pixel, double depth){
    /* rs_pixel2point
     * This function calculate the 3D point coordinates with the given 2D pixel & depth value & camera parameters
     * - Input
     *      - pixel[cv::Point2i]:, the x (column) & y (row) coordinates of the pixel
     *      - depth[double]: distance in mm of the pixel
     *      - CamParas[struct CameraParameters]: calibration parameters of the camera (Here, we align the depth image onto the color image space)
     * - Output
     *      - cv::Vec3f point: x & y & z coordinates of the 3D point
     * */
    cv::Vec3f point;

    point[0] = static_cast<float>((pixel.x - mCamParas.ppx)*depth/mCamParas.fx);
    point[1] = static_cast<float>((pixel.y - mCamParas.ppy)*depth/mCamParas.fy);
    point[2] = static_cast<float>(depth);

    return point;
}

cv::Point2i OpDetector::rs_point2pixel(cv::Vec3f point){
    /* rs_pixel2point
     * This function calculate the 2D pixel given the 3D point coordinates & camera parameters
     * - Input
     *      - point[cv::Vec3f]:, x & y & z coordinates of the 3D point
     *      - CamParas[struct CameraParameters]: calibration parameters of the camera (Here, we align the depth image onto the color image space)
     * - Output
     *      - cv::Point2i pixel: x & y & z coordinates of the 3D point
     * */
    cv::Point2i pixel;
    float X, Y;
    if (point[2]==0){
        X = 0.0; Y=0.0;
    }
    else{
        X = point[0]/point[2];
        Y = point[1]/point[2];
    }

    pixel.x = static_cast<int>(X*mCamParas.fx + mCamParas.ppx);
    pixel.y = static_cast<int>(Y*mCamParas.fy + mCamParas.ppy);
    return pixel;
}

cv::Mat OpDetector::Skeleton2Dto3D(cv::Mat& Joints2D, cv::Mat& imD, double renderThres){
    cv::Mat Points3D = cv::Mat::zeros(1,25,CV_32FC3); // Only 3D positions of joints

    for(int i = 0; i < 25; i++){
        Vec3f joint2d, joint3d;
        Point2i pixel;
        double z = 0.0;
        joint2d = Joints2D.at<Vec3f>(i);
        if ( joint2d[2] > renderThres){
            pixel.x = static_cast<int>(joint2d[0]);
            pixel.y = static_cast<int>(joint2d[1]);

            if (pixel.x >= 0 && pixel.x <= mCamParas.width && pixel.y >= 0 && pixel.y <= mCamParas.height){
                z = GetPixelDepth(pixel, imD, 4)/1000.0;
                joint3d = rs_pixel2point(pixel, z);

                Points3D.at<Vec3f>(i) = joint3d;
            }
            else{
                Points3D.at<Vec3f>(i) = {0,0,0};
            }
        }
        else{
            Points3D.at<Vec3f>(i) = {0,0,0};
        }
    }
    return Points3D;
}

void OpDetector::SkeletonSquareMask(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat& outputIm){
    int Im_Width = imD.cols;
    int Im_Height = imD.rows;

    int XMIN = Im_Width - 1;
    int YMIN = Im_Height - 1;
    int XMAX = 0;
    int YMAX = 0;

    /*
    for (int i =8; i < 15; i++){
        XMIN = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[0] < XMIN ? Joints2D.at<cv::Vec3f>(i)[0] : XMIN);
        XMAX = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[0] > XMAX ? Joints2D.at<cv::Vec3f>(i)[0] : XMAX);
        YMIN = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[1] < YMIN ? Joints2D.at<cv::Vec3f>(i)[1] : YMIN);
        YMAX = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[1] > YMAX ? Joints2D.at<cv::Vec3f>(i)[1] : YMAX);
    }*/

    for (int i = 0; i < 25; i++){
            XMIN = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[0] < XMIN ? Joints2D.at<cv::Vec3f>(i)[0] : XMIN);
            XMAX = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[0] > XMAX ? Joints2D.at<cv::Vec3f>(i)[0] : XMAX);
            YMIN = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[1] < YMIN ? Joints2D.at<cv::Vec3f>(i)[1] : YMIN);
            YMAX = static_cast<int>(Joints2D.at<cv::Vec3f>(i)[1] > YMAX ? Joints2D.at<cv::Vec3f>(i)[1] : YMAX);
    }

    XMIN = XMIN - 30; XMAX = XMAX + 30;
    YMIN = YMIN - 30; YMAX = YMAX + 30;

    mXMIN = XMIN <= 0 ? 0 : XMIN;
    mYMIN = YMIN <= 0 ? 0 : YMIN;
    mXMAX = XMAX >= Im_Width - 1 ? Im_Width - 1 : XMAX;
    mYMAX = YMAX >= Im_Height - 1? Im_Height - 1: YMAX;

    //cout << "FUNC " << mYMIN << " "<< mYMAX << " "<< mXMIN << " "<< mXMAX << endl;
    //cout << outputIm.rows << " " << outputIm.cols << endl;
    cv::Scalar maskValue = cv::Scalar(0);
    outputIm.rowRange(mYMIN,mYMAX).colRange(mXMIN,mXMAX).setTo(maskValue);
}

void OpDetector::Skeleton3DSeg(cv::Mat& imD, cv::Mat& Joints2D, cv::Mat& Joints3D, cv::Mat& outputIm){
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
            rHead_px = (int)((0.15/(Z_max))*mCamParas.fx);
        else
            rHead_px = (int)((0.15/(p_z))*mCamParas.fx);

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
                rAxisX_px = (int)((rElp/p_z1)*mCamParas.fx); rAxisY_px = (int)((lElp/p_z1)*mCamParas.fx); }
            else if (p_z1 == 0 && p_z2 > 0){
                rAxisX_px = (int)((rElp/p_z2)*mCamParas.fx); rAxisY_px = (int)((lElp/p_z2)*mCamParas.fx); }
            else{
                rAxisX_px = (int)((2*rElp/(p_z1+p_z2))*mCamParas.fx); rAxisY_px = (int)((2*lElp/((p_z1+p_z2)))*mCamParas.fx);  }
        }
        else{
            rAxisX_px = rElp/Z_max*mCamParas.fx;  rAxisY_px = lElp/Z_max*mCamParas.fx;   }

        double angles;
        if (p_x2 == p_x1 )
            angles = 0.0;
        else
            angles = -3.1415/2.0 + atan((p_y2-p_y1)/(p_x2-p_x1));

        cv::ellipse(outputIm, Point((p_x1+p_x2)/2, (p_y1+p_y2)/2), Size(rAxisX_px, rAxisY_px), angles*180/3.1415, 0.0, 360.0, maskValue, -1);
    }

}

double OpDetector::GetPixelDepth(cv::Point2i pixel, cv::Mat& imD, int depth_radius){
    /*
     * GetPixelDepth: get the depth value of a given pixel. Due to that the depth image
     * typically contains several zero values. Hence, we need to estimate the depth value
     * according to its neighbors
     * Input:
     *     - point2D (x,y)
     *     - imD: depth image
     *     - depth_radius: radius of the neighborhood
     */
    double z = 0.0;
    int z_valid = 0, z_cnt = 0;
    int x = pixel.x;
    int y = pixel.y;
    int x_lb = x-depth_radius, x_ub = x+depth_radius;
    int y_lb = y-depth_radius, y_ub = y+depth_radius;

    // Bound limitation
    x_lb = x_lb<0 ? 0:x_lb;          x_lb = x_lb>imD.cols-1 ? imD.cols-1:x_lb;
    x_ub = x_ub<=x_lb ? x_lb+1:x_ub; x_ub = x_ub>imD.cols-1 ? imD.cols-1:x_ub;
    y_lb = y_lb<0 ? 0:y_lb;          y_lb = y_lb>imD.rows-1 ? imD.rows-1:y_lb;
    y_ub = y_ub<=y_lb ? y_lb+1:y_ub; y_ub = y_ub>imD.rows-1 ? imD.rows-1:y_ub;

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
    /* Plot2DJoints
     *
     * */
    for(int i=0; i<20; i++){
        Vec3f jStart, jEnd;
        jStart = Joints2D.at<Vec3f>(mFullbodyPair[0][i]);
        jEnd = Joints2D.at<Vec3f>(mFullbodyPair[1][i]);
        Point2i jStartPlot, jEndPlot;
        float scoreStart, scoreEnd;
        jStartPlot.x = static_cast<int>(jStart[0]);
        jStartPlot.y = static_cast<int>(jStart[1]);
        scoreStart = jStart[2];
        jEndPlot.x = static_cast<int>(jEnd[0]);
        jEndPlot.y = static_cast<int>(jEnd[1]);
        scoreEnd = jEnd[2];
        if (scoreStart > renderThres && scoreEnd > renderThres){
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

cv::KalmanFilter OpDetector::KFinit(struct KalmanFilterParams *KFparams){

    KalmanFilter KF(KFparams->stateNum, KFparams->measureNum, 0);
    Mat state(KFparams->stateNum, 1, CV_32FC1); // STATE (x, y, dx, dy)
    Mat processNoise(KFparams->stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(KFparams->measureNum, 1, CV_32F); //measurement(x,y)

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, Scalar::all(KFparams->wk));
    cv::setIdentity(KF.measurementNoiseCov, Scalar::all(KFparams->vk));
    cv::setIdentity(KF.errorCovPost, Scalar::all(KFparams->pk));

    Mat transitionMatrix = cv::Mat::eye(KFparams->stateNum, KFparams->stateNum, CV_32FC1);


    for (int i = 0; i < KFparams->measureNum; i++){
        transitionMatrix.at<float>(i, KFparams->measureNum + i) = 1;
    }

    KF.transitionMatrix = transitionMatrix;

    return KF;
}

cv::Mat OpDetector::KFupdate(cv::Mat &Joints3D, const int stateNum, const int measureNum){

    cv::Mat Joints3D_KFsmooth;
    Joints3D_KFsmooth = Joints3D.clone();

    /// KALMAN SMOOTHER
    for (int i = 0; i < 13; i ++){
        int idx = mLowerLimbSet[i];
        Vec3f jointRaw = Joints3D_KFsmooth.at<cv::Vec3f>(idx);
        Vec3f jointSmooth;
        //cout << KFs3D[idx].processNoiseCov << endl;
        //cout << KFs3D[idx].measurementNoiseCov << endl;
        Mat prediction = KFs3D[idx].predict().t();
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
        KFs3D[idx].statePost = estimatedPt;

        // Refinement according to the human body constraints
        Mat updatePt;
        if (idx == HIP_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(HIP_C), mHumanParams.Link_hip_L);
        else if (idx == HIP_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(HIP_C), mHumanParams.Link_hip_R);
        else if (idx == KNEE_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(HIP_R), mHumanParams.Link_thigh_R);
        else if (idx == KNEE_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(HIP_L), mHumanParams.Link_thigh_L);
        else if (idx == ANKLE_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(KNEE_R), mHumanParams.Link_shank_R);
        else if (idx == ANKLE_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(KNEE_L), mHumanParams.Link_shank_L);
        else if (idx == TOE_IN_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_foot_R);
        else if (idx == TOE_OUT_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_foot_R);
        else if (idx == HEEL_R)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_R), mHumanParams.Link_heel_R);
        else if (idx == TOE_IN_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_foot_L);
        else if (idx == TOE_OUT_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_foot_L);
        else if (idx == HEEL_L)
            updatePt = SmoothWithLengthConstraint(estimatedPt.rowRange(0,3), Joints3D_KFsmooth.at<cv::Vec3f>(ANKLE_L), mHumanParams.Link_heel_L);

        // Final output
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

void OpDetector::KFupdate(cv::Mat &Joints3D, cv::Mat &JointsLastFrame, const int stateNum, const int measureNum){
    /*
     * Calculate the distance between two adjacent skeletons to determine whether the skeleton data is reliable or not
     * */
    unsigned int SetSize = sizeof(mLowerLimbSet)/ sizeof(mLowerLimbSet[0]);
    double distSkel = SkeletonDist(Joints3D, JointsLastFrame, mLowerLimbSet, SetSize);

    //std::cout << "distance between skeleton is: " << distSkel << std::endl;

}

double OpDetector::SkeletonDist(cv::Mat &skel_curr, cv::Mat &skel_last, int *JointSet, const unsigned int SetSize){
    /* CalcSkelDist:
     * This function calculate the Euclidean distance between two 3D skeleton.
     * -- Input:
     *      - skel_curr: 3D skeleton in current frame: J = [N*1*3] (cv::Vec3f)
     *      - skel_last: 3D skeleton in last frame:    J = [N*1*3] (cv::Vec3f)
     *      - JointSet: selected joints needed to calculate the distance e.g., {Joints of lower body} or {Joints of full body}
     * -- Output:
     *      - SkelDist (float): Euclidean distance
     */

    double SkelDist = 0.0;

    for (unsigned int i = 0; i < SetSize; i++){
        int idx = JointSet[i];
        // Only calculate the distance between two valid joints
        //if (skel_curr.at<cv::Vec3f>(idx)[2]>0 && skel_last.at<cv::Vec3f>(idx)[2] >0)

        SkelDist = SkelDist + cv::norm(skel_curr.at<cv::Vec3f>(idx), skel_last.at<cv::Vec3f>(idx));
    }
    return SkelDist;
}

void OpDetector::InitHumanParams(){
    /* InitHumanParams:
     * This function is to initialize the parameters of the human body skeleton
     * */
    mHumanParams.Link_thigh_L = 0.35;
    mHumanParams.Link_thigh_R = 0.35;
    mHumanParams.Link_shank_L = 0.35;
    mHumanParams.Link_shank_R = 0.35;
    mHumanParams.Link_foot_L = 0.01;
    mHumanParams.Link_foot_R = 0.01;
    mHumanParams.Link_hip_L = 0.25;
    mHumanParams.Link_hip_R = 0.25;
    mHumanParams.Link_heel_L = 0.1;
    mHumanParams.Link_heel_R = 0.1;

    /*
    mHumanParams.link_heel_MAX = 0.15;     mHumanParams.link_heel_MIN = 0.05;
    mHumanParams.link_foot_MAX = 0.15;     mHumanParams.link_foot_MIN = 0.05;
    mHumanParams.link_shank_MAX = 0.5;    mHumanParams.link_shank_MIN = 0.2;
    mHumanParams.link_thigh_MAX = 0.5;    mHumanParams.link_thigh_MIN = 0.2;
    mHumanParams.link_hip_MAX = 0.3;      mHumanParams.link_hip_MIN = 0.15;
     */
    mHumanParams.link_heel_MAX = 0.1;     mHumanParams.link_heel_MIN = 0.1;
    mHumanParams.link_foot_MAX = 0.1;     mHumanParams.link_foot_MIN = 0.1;
    mHumanParams.link_shank_MAX = 0.4;    mHumanParams.link_shank_MIN = 0.3;
    mHumanParams.link_thigh_MAX = 0.4;    mHumanParams.link_thigh_MIN = 0.3;
    mHumanParams.link_hip_MAX = 0.3;      mHumanParams.link_hip_MIN = 0.2;
}

void OpDetector::UpdateHumanParams_LinkLength(cv::Mat &Joints3D, const int window_size){
    // i - segments index
    for (int i = 0; i < 12; i ++){
        cv::Vec3f P1 = Joints3D.at<cv::Vec3f>(mLowerPair[0][i]);
        cv::Vec3f P2 = Joints3D.at<cv::Vec3f>(mLowerPair[1][i]);
        double LinkLength;
        switch (i){
            case 0: // HIP_R
                if (P1[2] > 0 && P2[2] > 0){ // Two points have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_hip_MAX ? LinkLength : mHumanParams.link_hip_MAX;
                    LinkLength = LinkLength > mHumanParams.link_hip_MIN ? LinkLength : mHumanParams.link_hip_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_hip_R > mHumanParams.link_hip_MIN)
                        mHumanParams.Link_hip_R = LinkLength/window_size + mHumanParams.Link_hip_R - mHumanParams.Link_hip_R/window_size;
                    else
                        mHumanParams.Link_hip_R = LinkLength;
                    break;
                }
            case 1: // HIP_L
                if (P1[2] > 0 && P2[2]> 0){ // Two points have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_hip_MAX ? LinkLength : mHumanParams.link_hip_MAX;
                    LinkLength = LinkLength > mHumanParams.link_hip_MIN ? LinkLength : mHumanParams.link_hip_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_hip_L > mHumanParams.link_hip_MIN)
                        mHumanParams.Link_hip_L = LinkLength/window_size + mHumanParams.Link_hip_L - mHumanParams.Link_hip_L/window_size;
                    else
                        mHumanParams.Link_hip_L = LinkLength;
                    break;
                }

            case 2: // THIGH_R
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_thigh_MAX ? LinkLength : mHumanParams.link_thigh_MAX;
                    LinkLength = LinkLength > mHumanParams.link_thigh_MIN ? LinkLength : mHumanParams.link_thigh_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_thigh_R > mHumanParams.link_thigh_MIN)
                        mHumanParams.Link_thigh_R = LinkLength/window_size + mHumanParams.Link_thigh_R - mHumanParams.Link_thigh_R/window_size;
                    else
                        mHumanParams.Link_thigh_R = LinkLength;
                    break;
                }
            case 3: // SHANK_R
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_shank_MAX ? LinkLength : mHumanParams.link_shank_MAX;
                    LinkLength = LinkLength > mHumanParams.link_shank_MIN ? LinkLength : mHumanParams.link_shank_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_shank_R > mHumanParams.link_shank_MIN)
                        mHumanParams.Link_shank_R = LinkLength/window_size + mHumanParams.Link_shank_R - mHumanParams.Link_shank_R/window_size;
                    else
                        mHumanParams.Link_shank_R = LinkLength;
                    break;
                }
            case 4: case 5: // FOOT_R
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_foot_MAX ? LinkLength : mHumanParams.link_foot_MAX;
                    LinkLength = LinkLength > mHumanParams.link_foot_MIN ? LinkLength : mHumanParams.link_foot_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_foot_R > mHumanParams.link_foot_MIN)
                        mHumanParams.Link_foot_R = LinkLength/window_size + mHumanParams.Link_foot_R - mHumanParams.Link_foot_R/window_size;
                    else
                        mHumanParams.Link_foot_R = LinkLength;
                    break;
                }
            case 6: // HEEL_R
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_heel_MAX ? LinkLength : mHumanParams.link_heel_MAX;
                    LinkLength = LinkLength > mHumanParams.link_heel_MIN ? LinkLength : mHumanParams.link_heel_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_heel_R > mHumanParams.link_heel_MIN)
                        mHumanParams.Link_heel_R = LinkLength/window_size + mHumanParams.Link_heel_R - mHumanParams.Link_heel_R/window_size;
                    else
                        mHumanParams.Link_heel_R = LinkLength;
                    break;
                }
            case 7: //THIGH_L
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_thigh_MAX ? LinkLength : mHumanParams.link_thigh_MAX;
                    LinkLength = LinkLength > mHumanParams.link_thigh_MIN ? LinkLength : mHumanParams.link_thigh_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_thigh_L > mHumanParams.link_thigh_MIN)
                        mHumanParams.Link_thigh_L = LinkLength/window_size + mHumanParams.Link_thigh_L - mHumanParams.Link_thigh_L/window_size;
                    else
                        mHumanParams.Link_thigh_L = LinkLength;
                    break;
                }
            case 8: //SHANK_L
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_shank_MAX ? LinkLength : mHumanParams.link_shank_MAX;
                    LinkLength = LinkLength > mHumanParams.link_shank_MIN ? LinkLength : mHumanParams.link_shank_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_shank_L > mHumanParams.link_shank_MIN)
                        mHumanParams.Link_shank_L = LinkLength/window_size + mHumanParams.Link_shank_L - mHumanParams.Link_shank_L/window_size;
                    else
                        mHumanParams.Link_shank_L = LinkLength;
                    break;
                }
            case 9: case 10: // FOOT_L
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_foot_MAX ? LinkLength : mHumanParams.link_foot_MAX;
                    LinkLength = LinkLength > mHumanParams.link_foot_MIN ? LinkLength : mHumanParams.link_foot_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_foot_L > mHumanParams.link_foot_MIN)
                        mHumanParams.Link_foot_L = LinkLength/window_size + mHumanParams.Link_foot_L - mHumanParams.Link_foot_L/window_size;
                    else
                        mHumanParams.Link_foot_L = LinkLength;
                    break;
                }
            case 11: // HEEL_L
                if (P1[2] > 0 && P2[2] > 0){ // have valid depth value
                    LinkLength = cv::norm(P1, P2, cv::NORM_L2);
                    // MIN-MAX smooth
                    LinkLength = LinkLength < mHumanParams.link_heel_MAX ? LinkLength : mHumanParams.link_heel_MAX;
                    LinkLength = LinkLength > mHumanParams.link_heel_MIN ? LinkLength : mHumanParams.link_heel_MIN;
                    // Moving average smooth
                    if (mHumanParams.Link_heel_L > mHumanParams.link_heel_MIN)
                        mHumanParams.Link_heel_L = LinkLength/window_size + mHumanParams.Link_heel_L - mHumanParams.Link_heel_L/window_size;
                    else
                        mHumanParams.Link_heel_L = LinkLength;
                    break;
                }
                //std::cout << "Default: [i] is " << i << std::endl;
        }
    }
}

cv::Mat OpDetector::SmoothWithLengthConstraint(cv::Mat measurementPt, cv::Vec3f parentPt, float linkConstraint){
    /* SmoothWithLengthConstraint
     * This function is to smooth the joint positions using the link length constraint
     * -- Input:
     *      - measurementPt: the 3D position of the measurement point
     *      - parentPt: the parent point 
     * */
    Mat correctPt;
    correctPt = measurementPt.clone();
    if (linkConstraint > 0){
        Mat parentPtmat = Mat::zeros(3,1,CV_32F);
        parentPtmat.at<float>(0) = parentPt[0];
        parentPtmat.at<float>(1) = parentPt[1];
        parentPtmat.at<float>(2) = parentPt[2];
        float vec_x = correctPt.at<float>(0) - parentPtmat.at<float>(0);
        float vec_y = correctPt.at<float>(1) - parentPtmat.at<float>(1);
        float vec_z = correctPt.at<float>(2) - parentPtmat.at<float>(2);
        float linkLength = (float)(cv::norm(correctPt, parentPtmat, cv::NORM_L2));

        if (linkLength > linkConstraint){
            correctPt.at<float>(0) = parentPtmat.at<float>(0) + sqrt(linkConstraint/linkLength)*vec_x;
            correctPt.at<float>(1) = parentPtmat.at<float>(1) + sqrt(linkConstraint/linkLength)*vec_y;
            correctPt.at<float>(2) = parentPtmat.at<float>(2) + sqrt(linkConstraint/linkLength)*vec_z;
            //cout << "Data: " << linkLength << " " << linkConstraint << " " << measurementPt << " " << updatePt << " " << rootPtmat << endl;
        }
    }
    return correctPt;
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