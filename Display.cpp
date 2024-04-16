#include "Display.h"

Display::Display(Camera camera, cv::Size frameSize, double scale) : 
	camera(camera), frameSize(frameSize), scale(scale) {
	cv::namedWindow("Display");
	display = cv::Mat(cv::Size(frameSize.width*2, frameSize.height*2), CV_8UC3);	
	display.setTo(cv::Scalar(0, 0, 0));
	// setup cube points
	cubePoints.resize(8);
	for (int x = 0; x < 2; ++x) {
		for (int y = 0; y < 2; ++y) {
			for (int z = 0; z < 2; ++z) {
				cubePoints[4*x+2*y+z] = cv::Point3d(x*10, y*10, -z*10);	
			}
		}
	}
}

void Display::show(const cv::Mat& frame, const Pose& pose) {
	cv::Mat topLeft(display, cv::Rect(0, 0, frameSize.width, frameSize.height));
	cv::Mat topRight(display, cv::Rect(frameSize.width, 0, frameSize.width, frameSize.height));
	cv::Mat bottomLeft(display, cv::Rect(0, frameSize.height, frameSize.width, frameSize.height));
	cv::Mat bottomRight(display, cv::Rect(frameSize.width, frameSize.height, frameSize.width, frameSize.height));
	frame.copyTo(topLeft);
	cv::Mat R;
	cv::Rodrigues(pose.rvec, R);
	cv::Mat c = -R.t() * pose.tvec;
	if (pose.valid) {

		// draw cube
		std::vector<cv::Point2d> imagePoints;
		cv::projectPoints(cubePoints, pose.rvec, pose.tvec, camera.K, camera.d, imagePoints);
		for (const auto& p : imagePoints) {
			cv::circle(topLeft, p, 5, cv::Scalar(255, 255, 255), -1);
		}	
		cv::line(topLeft, imagePoints[0], imagePoints[1], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[1], imagePoints[3], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[3], imagePoints[2], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[2], imagePoints[0], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[4], imagePoints[5], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[5], imagePoints[7], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[7], imagePoints[6], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[6], imagePoints[4], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[0], imagePoints[4], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[1], imagePoints[5], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[2], imagePoints[6], cv::Scalar(255, 255, 255), 3);
		cv::line(topLeft, imagePoints[3], imagePoints[7], cv::Scalar(255, 255, 255), 3);


		// draw path
		double cx = c.at<double>(0) * scale + frameSize.width / 2;
		double cy = c.at<double>(1) * scale + frameSize.height / 2;
		double cz = c.at<double>(2) * scale + frameSize.height - 20;
		cv::circle(bottomRight, cv::Point2d(cx, cy), 3, cv::Scalar(0, 255, 0), -1);
		cv::circle(bottomLeft, cv::Point2d(cx, cz), 3, cv::Scalar(0, 255, 0), -1);
		cy = c.at<double>(1) * scale + frameSize.width / 2;
		cv::circle(topRight, cv::Point2d(cy, cz), 3, cv::Scalar(0, 255, 0), -1);
		//cv::imwrite("output.jpg", display);
		//cv::imshow("Display", display);
		//cv::waitKey(0);
	}
	cv::imshow("Display", display);
}