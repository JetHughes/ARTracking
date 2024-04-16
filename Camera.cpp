#include "Camera.h"
#include <fstream>

Camera::Camera() : 
	K(cv::Mat::eye(3, 3, CV_64F)), d() { 

}


std::ostream& operator<<(std::ostream& os, const Camera& cam) {
	for (int r = 0; cam.K.rows; ++r) {
		for (int c = 0; c < cam.K.cols; ++c) {
			os << cam.K.at<double>(r, c) << " ";
		}
		os << std::endl;
	}
	os << cam.d.size() << std::endl;
	for (const auto& v : cam.d) {
		os << v << " ";
	}
	os << std::endl;
	return os;
}

std::istream& operator>>(std::istream& is, Camera& cam) {
	for (int r = 0; r < cam.K.rows; ++r) {
		for (int c = 0; c < cam.K.cols; ++c) {
			is >> cam.K.at<double>(r, c);
		}
	}
	size_t numD;
	is >> numD;
	cam.d.resize(numD);
	for (auto& v : cam.d) {
		is >> v;
	}
	return is;
}
