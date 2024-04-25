#pragma once

#include "PoseEstimator.h"

class ImprovedPoseEstimator : public PoseEstimator {

public:
	ImprovedPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ImprovedPoseEstimator() = default;
	ImprovedPoseEstimator(const ImprovedPoseEstimator& other) = default;
	ImprovedPoseEstimator& operator=(const ImprovedPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;

private:
	Pose getInitialPose(const cv::Mat& img);
	Pose opticalFlowTracking(const cv::Mat& img);

	cv::Mat image;
	double imageWidth;

	// For absolute pose estimation
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;

	// For relative pose estimation
	cv::Mat prevFrame;
	cv::Mat H;
	Pose prevPose;
	std::vector<cv::Point3f> refFrameObjectPoints;
	std::vector<cv::Point2f> refFrameImagePoints;
	std::vector<cv::Point2f> trackingFeatures;

	// For drawing optical flow
	cv::Mat mask;
	std::vector<cv::Scalar> colors;

};