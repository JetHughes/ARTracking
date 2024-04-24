#pragma once

#include <opencv2/opencv.hpp>
#include "Pose.h"
#include "Camera.h"

using namespace std;
using namespace cv;

class Util {
public:
    Util();
    ~Util();

    static Pose solvePose(vector<Point2f> goodpoints2D, vector<Point3f> goodpoints3D, Camera camera);
    static double computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs);

private:
};
