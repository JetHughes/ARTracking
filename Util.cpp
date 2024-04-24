#include "util.h"

Util::Util() {}

Util::~Util() {}

Pose Util::solvePose(vector<Point2f> goodpoints2D, vector<Point3f> goodpoints3D, Camera camera) {
    Pose pose;
    pose.valid = false;

    if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) {
        double reprojectionError = computeReprojectionError(goodpoints3D, goodpoints2D, pose.rvec, pose.tvec, camera.K, camera.d);
        if (reprojectionError < 10) {
            pose.valid = true;
        }
        pose.err = reprojectionError;
    }

    return pose;
}

double Util::computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs) {
	std::vector<cv::Point2f> projectedPoints;
	projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoeffs, projectedPoints);

	double totalError = 0.0;
	for (size_t i = 0; i < imagePoints.size(); ++i)
	{
		double dx = projectedPoints[i].x - imagePoints[i].x;
		double dy = projectedPoints[i].y - imagePoints[i].y;
		double error = sqrt(dx * dx + dy * dy);
		totalError += error;
	}

	double meanError = totalError / imagePoints.size();
	return meanError;
}