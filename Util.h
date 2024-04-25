#pragma once

#include <opencv2/opencv.hpp>
#include "Pose.h"
#include "Camera.h"

class Util {
public:
    Util();
    ~Util();

    static Pose solvePose(const std::vector<cv::Point2f>& goodpoints2D, const std::vector<cv::Point3f>& goodpoints3D, const Camera& camera, double maxErr=10);
    static double computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const Pose& pose, const Camera& camera);
    static float round2dp(float num);

private:
};
