#pragma once

#include "PoseEstimator.h"

class ImagePoseEstimator : public PoseEstimator {

public:
	ImagePoseEstimator(const Camera& camera, std::string imageFile, double imageWidth);
	virtual ~ImagePoseEstimator() = default;
	ImagePoseEstimator(const ImagePoseEstimator& other) = default;
	ImagePoseEstimator& operator=(const ImagePoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;

private:
	cv::Mat image;
	double imageWidth;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Point3f> objectPoints;
};