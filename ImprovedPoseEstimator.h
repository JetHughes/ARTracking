#pragma once

#include "PoseEstimator.h"

class ImprovedPoseEstimator : public PoseEstimator {

public:
	ImprovedPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ImprovedPoseEstimator() = default;
	ImprovedPoseEstimator(const ImprovedPoseEstimator& other) = default;
	ImprovedPoseEstimator& operator=(const ImprovedPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;
	Pose ImprovedPoseEstimator::estimatePoseHomo(const cv::Mat& img);

private:
	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;
	cv::Mat prevFrame;
};