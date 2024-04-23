#pragma once

#include "PoseEstimator.h"

class ImprovedPoseEstimator : public PoseEstimator {

public:
	ImprovedPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ImprovedPoseEstimator() = default;
	ImprovedPoseEstimator(const ImprovedPoseEstimator& other) = default;
	ImprovedPoseEstimator& operator=(const ImprovedPoseEstimator& other) = default;
	double ImprovedPoseEstimator::computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs);

	Pose estimatePose(const cv::Mat& img) override;

private:
	Pose ImprovedPoseEstimator::fullDetection(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors);
	Pose ImprovedPoseEstimator::relativePoseEstimation(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors);

	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::SIFT> detector, detectorH;
	cv::Ptr<cv::DescriptorMatcher> matcher, matcherH;
	std::vector<cv::Point3f> objectPoints;
	cv::Mat prevFrame;
	cv::Mat prevFrameDescriptors;
	std::vector<cv::KeyPoint> prevFrameKeypoints;
	Pose prevPose;
};