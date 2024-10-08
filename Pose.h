#pragma once

#include <opencv2/opencv.hpp>

class Pose {
public:

	Pose() : rvec(cv::Mat::zeros(3, 1, CV_64F)), tvec(cv::Mat::zeros(3, 1, CV_64F)), valid(false), err(-1) {}
	~Pose() = default;
	Pose(const Pose&) = default;
	Pose& operator=(const Pose&) = default;

	void toString() {
		//round to 4 decimal places
		std::cout << "Pose: " << std::setprecision(4) <<
		std::round(rvec.at<float>(0) * 10000) / 10000 << " " <<
		std::round(rvec.at<float>(1) * 10000) / 10000 << " " <<
		std::round(rvec.at<float>(2) * 10000) / 10000 << " " <<
		std::round(tvec.at<float>(0) * 10000) / 10000 << " " <<
		std::round(tvec.at<float>(1) * 10000) / 10000 << " " <<
		std::round(tvec.at<float>(2) * 10000) / 10000;
	}

	cv::Mat rvec;
	cv::Mat tvec;
	double err;
	bool valid;
};