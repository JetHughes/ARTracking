#pragma once

#include <opencv2/opencv.hpp>

class Pose {
public:

	Pose() : rvec(cv::Mat::zeros(3, 1, CV_64F)), tvec(cv::Mat::zeros(3, 1, CV_64F)), valid(false) {}
	~Pose() = default;
	Pose(const Pose&) = default;
	Pose& operator=(const Pose&) = default;

	void toString() {
		//round to 4 decimal places
		std::cout << std::setprecision(4) << 
		std::round(rvec.at<double>(0)*10000)/10000 << " " << 
		std::round(rvec.at<double>(1)*10000)/10000 << " " << 
		std::round(rvec.at<double>(2)*10000)/10000 << " " << 
		std::round(tvec.at<double>(0)*10000)/10000 << " " << 
		std::round(tvec.at<double>(1)*10000)/10000 << " " << 
		std::round(tvec.at<double>(2)*10000)/10000 << std::endl;
	}

	cv::Mat rvec;
	cv::Mat tvec;
	bool valid;
};