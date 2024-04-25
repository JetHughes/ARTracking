#pragma once

#include "PoseEstimator.h"

class FiducialPoseEstimator : public PoseEstimator {

public:
	FiducialPoseEstimator(const Camera& camera);
	virtual ~FiducialPoseEstimator() = default;
	FiducialPoseEstimator(const FiducialPoseEstimator& other) = default;
	FiducialPoseEstimator& operator=(const FiducialPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;

private:
	cv::aruco::Board board;
	cv::aruco::ArucoDetector detector;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();;

	double markerLength = 1.7;
	std::vector<int> markerIds = { 0, 1, 2, 3, 4, 5, 6, 7 };
	std::vector<cv::Point2f> markerLocations = {
		cv::Point2f(0, 0),
		cv::Point2f(8.48, 0),
		cv::Point2f(17.65, 0),
		cv::Point2f(17.65, 12.91),
		cv::Point2f(17.65, 24.66),
		cv::Point2f(17.65, 24.63),
		cv::Point2f(0, 24.63),
		cv::Point2f(0, 12.99)
	};

	std::vector<std::vector<cv::Point3f>> objPoints;
};