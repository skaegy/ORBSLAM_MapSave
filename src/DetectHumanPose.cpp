//
// Created by skaegy on 16/08/18.
//
#include "DetectHumanPose.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

OpDetector::OpDetector(const string &strOpenposeSettingsFile, const bool bHumanPose){
    if (bHumanPose){
        cv::FileStorage fs(strOpenposeSettingsFile, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            cerr << "Unable to open openpose parameter file!" << endl;
        }

        // int
        logging_level = fs["logging_level"];
        num_gpu_start = fs["num_gpu_start"];
        scale_number = fs["scale_number"];
        // double
        scale_gap = fs["scale_gap"];
        render_threshold = fs["render_threshold"];
        alpha_pose = fs["alpha_pose"];
        // string
        const string var_model_pose = fs["model_pose"];
        model_pose = var_model_pose;
        const string var_model_folder = fs["model_folder"];
        model_folder = var_model_folder;
        const int net_resolution_row = fs["net_resolution_row"];
        const int net_resolution_col = fs["net_resolution_col"];
        net_resolution = to_string(net_resolution_row)+"x"+to_string(net_resolution_col);
        const int output_resolution_row = fs["output_resolution_row"];
        const int output_resolution_col = fs["output_resolution_col"];
        output_resolution = to_string(output_resolution_row)+"x"+to_string(output_resolution_col);
        fs.release();
    }
}

void OpDetector::Run() {

    // ------------------------- INITIALIZATION -------------------------
    // Step 1 - Set logging level
    // - 0 will output all the logging messages
    // - 255 will output nothing
    op::check(0 <= logging_level && logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    const auto outputSize = op::flagsToPoint(output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(net_resolution, "-1x368");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(model_pose);
    //double lastTimeSec = 0.0;
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
    op::PoseCpuRenderer poseRenderer{poseModel, (float)render_threshold, true, (float)alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat;
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();

    while (!mbStopped) {
        if (mlLoadImage.size()>0)
        {
            cv::Mat inputImage = mlLoadImage.back();
            mlLoadImage.pop_back();

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
            // Step 4 - Estimate poseKeypoints
            poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
            // Step 5 - Render poseKeypoints
            poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);
            // Step 6 - OpenPose output format to cv::Mat
            auto OutputImage = opOutputToCvMat.formatToCvMat(outputArray);
            mlRenderPoseImage.push_front(OutputImage);
        }
    }
}

void OpDetector::SetViewer(ORB_SLAM2::Viewer *pViewer) {
    mpViewer = pViewer;
}

void OpDetector::OpLoadImage(const cv::Mat &im, const double &timestamp) {
    cv::Mat BufMat;
    im.copyTo(BufMat);
    mlLoadImage.push_front(BufMat);
}


void OpDetector::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool OpDetector::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void OpDetector::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool OpDetector::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void OpDetector::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool OpDetector::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool OpDetector::Stop()
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

void OpDetector::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

} // ORB_SLAM2