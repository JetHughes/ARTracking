#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

#include "Camera.h"
#include "CheckerboardPoseEstimator.h"
#include "FiducialPoseEstimator.h"
#include "ImprovedPoseEstimator.h"
#include "ORBPoseEstimator.h"
#include "Config.h"
#include "Display.h"
#include "ImagePoseEstimator.h"
#include "Timer.h"
#include "Util.h"


int main(int argc, char* argv[]) {

	// Read in the configuration file
	Config cfg; 
	if (!cfg.parse(argc, argv)) {
		cfg.printUsage(std::cerr);
		return -1;
	}

	// Create a camera object and read in the calibration file
	Camera camera;

	//Check calibrationFile exits
	std::ifstream calibFile(cfg.calibrationFile);
	if (!calibFile.is_open()) {
		std::cerr << "Error: Unable to open calibration file" << std::endl;
		return -1;
	}

	calibFile >> camera;
	calibFile.close();

	// Open the video source
	cv::VideoCapture cap;
	if (cfg.captureSource == CAMERA) {
		cap.open(cfg.cameraIndex, cv::CAP_DSHOW);
	}
	else {
		cap.open(cfg.videoFile, cv::CAP_MSMF);
	}

	if (!cap.isOpened()) {
		std::cerr << "Error: Unable to open video source" << std::endl;
		return -1;
	}

	int noKey = cv::waitKey(1);
	cv::Mat frame;
	while (frame.empty()) {
		cap >> frame;
	}
	Display display(camera, frame.size(), 3);

	// Create a pose estimator object
	PoseEstimator* poseEstimator = 0;
	if (cfg.method == CHECKERBOARD) {
		poseEstimator = new CheckerboardPoseEstimator(camera, cv::Size(cfg.gridWidth, cfg.gridHeight), cfg.gridSize);
	}
	else if (cfg.method == IMAGE) {
		poseEstimator = new ImagePoseEstimator(camera, cfg.imageFile, cfg.imageWidth);
	}
	else if (cfg.method == FIDUCIAL) {
		poseEstimator = new FiducialPoseEstimator(camera);
	} 
	else if (cfg.method == IMPROVED) {
		poseEstimator = new ImprovedPoseEstimator(camera, cfg.imageFile, cfg.imageWidth);
	}
	else if (cfg.method == ORB) {
		poseEstimator = new ORBPoseEstimator(camera, cfg.imageFile, cfg.imageWidth);
	}
	else {
		std::cerr << "Invalid method" << std::endl;
		return -1;
	}

	// Main loop - read images, estimate pose, and update display.
	Timer timer;
	while (cap.read(frame) && cv::waitKey(1) == noKey) {
		timer.reset();
		Pose pose = poseEstimator->estimatePose(frame);
		double elapsed = timer.elapsed_ms();
		std::cout << elapsed / 1000.0 << "," << pose.err << std::endl;
		display.show(frame, pose);
	}

	return 0;
}

