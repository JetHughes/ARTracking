#pragma once

#include "PoseEstimator.h"

class ORBPoseEstimator : public PoseEstimator {

public:
	ORBPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ORBPoseEstimator() = default;
	ORBPoseEstimator(const ORBPoseEstimator& other) = default;
	ORBPoseEstimator& operator=(const ORBPoseEstimator& other) = default;
	Pose estimatePose(const cv::Mat& img) override;
private:
	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;

	Pose ORBPoseEstimator::refinePose(std::vector<cv::Point2f> goodpoints2D, std::vector<cv::Point3f> goodpoints3D);
};