#pragma once

#include "PoseEstimator.h"

class CheckerboardPoseEstimator : public PoseEstimator {

	public:

	CheckerboardPoseEstimator(const Camera& camera, cv::Size gridSize, double gridScale);
	virtual ~CheckerboardPoseEstimator() = default;
	CheckerboardPoseEstimator(const CheckerboardPoseEstimator& other) = default;
	CheckerboardPoseEstimator& operator=(const CheckerboardPoseEstimator& other) = default;

	Pose estimatePose(const cv::Mat& img) override;

private:
	cv::Size gridSize;
	std::vector<cv::Point3f> gridPoints;

};