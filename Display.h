#pragma once

#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "Pose.h"

class Display {

	public:

	Display(Camera camera, cv::Size frameSize, double scale);
	Display(const Display&) = delete;
	Display& operator=(const Display&) = delete;
	~Display() = default;

	void show(const cv::Mat& frame, const Pose& pose);

private:
	Camera camera;
	cv::Size frameSize;
	double scale;
	cv::Mat display;

	std::vector<cv::Point3d> cubePoints;

};
