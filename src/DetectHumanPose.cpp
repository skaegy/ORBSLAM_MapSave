//
// Created by skaegy on 16/08/18.
//
#include "DetectHumanPose.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

namespace ORB_SLAM2 {

OpDetector::OpDetector(const string &strOpenposeSettingsFile, const bool bHumanPose, const int SensorMode){
    mbHumanPose = bHumanPose;
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

    fx = fs["Camera.fx"];
    fy = fs["Camera.fy"];
    ppx = fs["Camera.cx"];
    ppy = fs["Camera.cy"];
    wk = fs["KF.wk"];
    vk = fs["KF.vk"];
    pk = fs["KF.pk"];

    fs.release();
    mSensor = SensorMode;
}

void OpDetector::Run() {
    const auto timerBegin = std::chrono::high_resolution_clock::now();
    double lastTimeSec = 0.0;
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

    // ------------------------- 3D --> KALMAN FILTER INITIALIZATION -------------------------
    const int stateNum = 6;
    const int measureNum = 3;
    for (int i = 0; i < 25; i ++){
        KFs3D[i] = KFInitialization(stateNum, measureNum, wk, vk, pk);
    }
    int LowerLimb[13]={8,9,10,11,12,13,14,19,20,21,22,23,24};


    while (!mbStopped ) {
        if (mbHumanPose && mlLoadImage.size() > 0) {
            cv::Mat inputImage = mlLoadImage.front();
            double timestamp = mlLoadTimestamp.front();

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


            if (joints2D.rows > 0){
                cv::Mat keyJoint2D =GetInformPersonJoint(joints2D, render_threshold, cv::Size(inputImage.cols, inputImage.rows));

                auto newposeKeypoints = cvMatToOpOutput.createArray(keyJoint2D, scaleInputToOutput, outputResolution);
                poseRenderer.renderPose(newoutputArray, newposeKeypoints, scaleInputToOutput);
                auto newOutputImage = opOutputToCvMat.formatToCvMat(newoutputArray);
                //Plot2DJoints(keyJoint2D, inputImage, render_threshold);

                keyJoint2D.copyTo(mJoints2D);

                mlRenderPoseImage.push_front(newOutputImage);
                if (mlRenderPoseImage.size() > 2)
                    mlRenderPoseImage.pop_back();

                // RGB-D (mSensor == 2)
                if (mlLoadDepth.size() > 0){
                    /// Get depth value and map to 3D skeleton
                    cv::Mat inputDepth = mlLoadDepth.front();
                    cv::Mat Joints3D = Joints2Dto3D(keyJoint2D, inputDepth, render_threshold);
                    mvJoints3Draw.push_back(Joints3D);

                    /// Remove suddenly flip of the skeleton by openpose
                    /*
                    if (mvJoints3Draw.size() > 1){
                        std::vector<cv::Mat>::iterator lit_last = mvJoints3Draw.end()-2;
                        cv::Mat skel_last = *lit_last;
                        Joints3D = RemoveSkelFlip(Joints3D, skel_last);
                    }
                     */
                    cv::Mat Joints3D_EKFsmooth;
                    Joints3D.copyTo(Joints3D_EKFsmooth);

                    /// KALMAN SMOOTHER
                    for (int i = 0; i < 13; i ++){
                        int idx = LowerLimb[i];
                        Vec3f jointRaw = Joints3D.at<cv::Vec3f>(idx);
                        Vec3f jointSmooth;
                        if (isAfterFirst[idx] == false){  // (Frame = 1)
                            if (jointRaw[2] > 0){
                                Mat predictionPt = KFs3D[idx].predict();
                                Mat measurementPt = Mat::zeros(measureNum, 1, CV_32F); //measurement(x,y)
                                measurementPt.at<float>(0) = jointRaw[0];
                                measurementPt.at<float>(1) = jointRaw[1];
                                measurementPt.at<float>(2) = jointRaw[2];
                                Mat estimatedPt = KFs3D[idx].correct(measurementPt);
                                KFs3D[idx].statePost = KFs3D[idx].statePre + KFs3D[idx].gain * KFs3D[idx].temp5;

                                isAfterFirst[idx] = true;
                            }
                        }
                        else{ // Frame > 1
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

                            jointSmooth[0] = KFs3D[idx].statePost.at<float>(0);
                            jointSmooth[1] = KFs3D[idx].statePost.at<float>(1);
                            jointSmooth[2] = KFs3D[idx].statePost.at<float>(2);
                            Joints3D_EKFsmooth.at<cv::Vec3f>(idx) = jointSmooth;
                        }
                    }
                    mvJoints3DEKF.push_back(Joints3D_EKFsmooth);
                    mvTimestamp.push_back(timestamp);
                }
            }
            else{
                mlRenderPoseImage.push_front(inputImage);
                if (mlRenderPoseImage.size() > 2)
                    mlRenderPoseImage.pop_back();
            }
            mFramecnt++;
        }
    }
}

void OpDetector::SetViewer(ORB_SLAM2::Viewer *pViewer) {
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
    cv::Mat BufRGB, BufDepth;
    imRGB.copyTo(BufRGB);
    imD.copyTo(BufDepth);
    mlLoadImage.push_front(BufRGB);
    if (mlLoadImage.size()>1)
        mlLoadImage.pop_back();
    mlLoadDepth.push_front(BufDepth);
    if (mlLoadDepth.size()>1)
        mlLoadDepth.pop_back();
    mlLoadTimestamp.push_front(timestamp);
    if (mlLoadTimestamp.size()>1)
        mlLoadTimestamp.pop_back();
}

cv::Mat OpDetector::Joints2Dto3D(cv::Mat Joints2D, cv::Mat& imD, double renderThres){
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

                point3d[0] = (point2d[0] - ppx)*z/fx;
                point3d[1] = (point2d[1] - ppy)*z/fy;
                point3d[2] = z;
                Points3D.at<Vec3f>(i) = point3d;
            }
        }
    }
    return Points3D;
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
    if (x_lb<0)           x_lb=0;
    if (x_ub>imD.cols-1)  x_ub=imD.cols-1;
    if (y_lb<0)           y_lb=0;
    if (y_ub>imD.rows-1)  y_ub=imD.rows-1;
    cv::Mat z_ROI = imD.rowRange(y_lb,y_ub).colRange(x_lb,x_ub);
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

cv::Mat OpDetector::RemoveSkelFlip(cv::Mat skel_curr, cv::Mat skel_last){
    cv::Mat outMat;
    int LowerLimb[12]={9,10,11,12,13,14};
    int N = (int)(sizeof(LowerLimb)/sizeof(LowerLimb[0]));
    int LowerPair[2][12]={{9,10,11,12,13,14,19,20,21,22,23,24},
                          {12,13,14,9,10,11,22,23,24,19,20,21}};

    cv::Mat tmp1 = skel_curr.colRange(0,9).clone();
    cv::Mat tmp2 = skel_curr.colRange(9,12).clone();
    cv::Mat tmp3 = skel_curr.colRange(12,15).clone();
    cv::Mat tmp4 = skel_curr.colRange(19,22).clone();
    cv::Mat tmp5 = skel_curr.colRange(22,25).clone();

    cv::Mat skel_currFlip;
    cv::hconcat(tmp1, tmp3, skel_currFlip);
    cv::hconcat(skel_currFlip, tmp2, skel_currFlip);
    cv::hconcat(skel_currFlip, tmp5, skel_currFlip);
    cv::hconcat(skel_currFlip, tmp4, skel_currFlip);

    cout << "Current" << endl;
    float DistStay = CalcSkelDist(skel_curr, skel_last, LowerLimb, N);
    cout << "Current Flip" << endl;
    float DistFlip = CalcSkelDist(skel_currFlip, skel_last, LowerLimb, N);
    cout << "Stay distance: " << DistStay << " DistFlip: " << DistFlip << endl;
    if (DistStay <= DistFlip)
        skel_curr.copyTo(outMat);
    else
        skel_currFlip.copyTo(outMat);

    return outMat;
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

            cout << idx << " " << skel_curr.at<cv::Vec3f>(idx) << " " << skel_last.at<cv::Vec3f>(idx) << " "
                    << cv::norm(skel_curr.at<cv::Vec3f>(idx), skel_last.at<cv::Vec3f>(idx)) << endl;
        }

    }
    return SkelDist;
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