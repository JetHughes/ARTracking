#pragma once

#include "PoseEstimator.h"

class ImprovedPoseEstimator : public PoseEstimator {

public:
	ImprovedPoseEstimator(const Camera& camera, std::string imageFile, double imageWidth, cv::Mat frame);
	virtual ~ImprovedPoseEstimator() = default;
	ImprovedPoseEstimator(const ImprovedPoseEstimator& other) = default;
	ImprovedPoseEstimator& operator=(const ImprovedPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;

private:
	Pose ImprovedPoseEstimator::fullDetection(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors);
	Pose ImprovedPoseEstimator::relativePoseEstimation(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors);
	Pose ImprovedPoseEstimator::opticalFlowTracking(const cv::Mat& img);

	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::SIFT> detector, detectorH;
	cv::Ptr<cv::DescriptorMatcher> matcher, matcherH;
	std::vector<cv::Point3f> objectPoints;
	cv::Mat prevFrame;
	cv::Mat prevFrameDescriptors;
	cv::Mat prevHomography;
	std::vector<cv::Point3f> refFrameObjectPoints;
	std::vector<cv::Point2f> refFrameImagePoints;
	std::vector<cv::KeyPoint> prevFrameKeypoints;
	std::vector<cv::Point2f> prevFrameImagePoints;
	cv::Mat mask;
	Pose prevPose;
};