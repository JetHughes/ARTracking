#include "CheckerboardPoseEstimator.h"

CheckerboardPoseEstimator::CheckerboardPoseEstimator(
	const Camera& camera, cv::Size gridSize, double gridScale) :
	PoseEstimator(camera), gridSize(gridSize), gridPoints() {

	for (int i = 0; i < gridSize.height; i++) {
		for (int j = 0; j < gridSize.width; j++) {
			gridPoints.push_back(cv::Point3f(j*gridScale, i*gridScale, 0));
		}
	}
}


Pose CheckerboardPoseEstimator::estimatePose(const cv::Mat& img) {
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	
	std::vector<cv::Point2f> corners;

	Pose pose;
	pose.valid = false;

	if (cv::findChessboardCorners(gray, gridSize, corners)) {
		if (cv::solvePnP(gridPoints, corners, camera.K, camera.d, pose.rvec, pose.tvec)) {
			pose.valid = true;
		}
	}

	return pose;
}