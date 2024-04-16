#pragma once

#include <opencv2/opencv.hpp>

class Pose {
public:

	Pose() : rvec(cv::Mat::zeros(3, 1, CV_64F)), tvec(cv::Mat::zeros(3, 1, CV_64F)), valid(false) {}
	~Pose() = default;
	Pose(const Pose&) = default;
	Pose& operator=(const Pose&) = default;

	cv::Mat rvec;
	cv::Mat tvec;
	bool valid;
};