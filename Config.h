#pragma once

#include <iostream>

enum PoseEstimationMethod {
	CHECKERBOARD,
	IMAGE,
	FIDUCIAL,
	IMPROVED,
	ORB
};

enum CaptureSource {
	CAMERA,
	VIDEO
};	

struct Config {

	bool parse(int argc, char *argv[]);
	void printUsage(std::ostream& os) const;

	CaptureSource captureSource;
	int cameraIndex;
	std::string videoFile;	

	std::string calibrationFile;
	
	PoseEstimationMethod method;

	int gridWidth;
	int gridHeight;
	double gridSize;

	double markerSize;
	
	std::string imageFile;
	double imageWidth;


};