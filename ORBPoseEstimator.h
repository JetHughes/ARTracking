#pragma once

#include "PoseEstimator.h"

class ORBPoseEstimator : public PoseEstimator {

public:
	ORBPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ORBPoseEstimator() = default;
	ORBPoseEstimator(const ORBPoseEstimator& other) = default;
	ORBPoseEstimator& operator=(const ORBPoseEstimator& other) = default;
	double ORBPoseEstimator::computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs);
	Pose estimatePose(const cv::Mat& img) override;

private:
	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;
};