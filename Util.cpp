#include "util.h"

using namespace std;
using namespace cv;

Util::Util() {}
Util::~Util() {}


// Solve the pose of the camera using solvepnp ransac given the 2D and 3D points
Pose Util::solvePose(const vector<Point2f>& goodpoints2D, const vector<Point3f>& goodpoints3D, const Camera& camera, double maxErr) {
    Pose pose;
    pose.valid = false;

    if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) {
        pose.err = computeReprojectionError(goodpoints3D, goodpoints2D, pose, camera);
        if (pose.err < maxErr) {
            pose.valid = true;
        }
    }

    return pose;
}

double Util::computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const Pose& pose, const Camera& camera) {
	std::vector<cv::Point2f> projectedPoints;
	projectPoints(objectPoints, pose.rvec, pose.tvec, camera.K, camera.d, projectedPoints);

	double sumError = 0.0;
	for (size_t i = 0; i < imagePoints.size(); ++i)
	{
		double error = cv::norm(imagePoints[i] - projectedPoints[i]);
		sumError += error;
	}

	double meanError = sumError / imagePoints.size();
	return meanError;
}

// from https://www.geeksforgeeks.org/rounding-floating-point-number-two-decimal-places-c-c/
float Util::round2dp(float num) {
	float value = (int)(num * 100 + .5);
	return (float)value / 100;
}