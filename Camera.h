#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class Camera {

public:

	Camera();
	Camera(const Camera&) = default;
	Camera& operator=(const Camera&) = default;
	~Camera() = default;

	cv::Mat K;
	std::vector<double> d;

};

std::ostream& operator<<(std::ostream& os, const Camera& cam);
std::istream& operator>>(std::istream& is, Camera& cam);