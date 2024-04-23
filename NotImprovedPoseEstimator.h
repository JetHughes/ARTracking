#pragma once

#include "PoseEstimator.h"

class NotImprovedPoseEstimator : public PoseEstimator {

public:
	NotImprovedPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~NotImprovedPoseEstimator() = default;
	NotImprovedPoseEstimator(const NotImprovedPoseEstimator& other) = default;
	NotImprovedPoseEstimator& operator=(const NotImprovedPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;
	Pose NotImprovedPoseEstimator::estimatePoseHomo(const cv::Mat& img);

private:
	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;
	cv::Mat prevFrame;
	Pose prevPose;
};