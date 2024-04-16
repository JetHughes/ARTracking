#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "Pose.h"

class PoseEstimator {

public:

	PoseEstimator(const Camera& cam) : camera(cam) {};
	virtual ~PoseEstimator() = default;
	PoseEstimator(const PoseEstimator& other) = default;
	PoseEstimator& operator=(const PoseEstimator& other) = default;

	virtual Pose estimatePose(const cv::Mat& img) = 0;

protected:

	Camera camera;

};