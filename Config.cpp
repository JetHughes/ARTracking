#include "Config.h"
#include <algorithm>
#include <string>

bool is_numeric(std::string str) {
    for (char c : str) {
        if (!std::isdigit(c)) {
			return false;
		}
	}
	return true;
}

bool Config::parse(int argc, char* argv[]) {
    if (argc != 6 && argc != 7 && argc != 5) {
        std::cerr << "Invalid number of arguments" << std::endl;
		return false;
	} 

    if (is_numeric(argv[1])) {
        captureSource = CAMERA;
		cameraIndex = std::stoi(argv[1]);
	} else {
		captureSource = VIDEO;
		videoFile = argv[1];
    }

    calibrationFile = argv[2];

    std::string methodStr = argv[3];
    std::transform(methodStr.begin(), methodStr.end(), methodStr.begin(), std::toupper);   

    if (methodStr == "CHECKERBOARD") {
		method = CHECKERBOARD;
        if (argc != 7) {
            std::cerr << "Invalid number of arguments" << std::endl;
			return false;
		}
		gridWidth = std::stoi(argv[4]);
		gridHeight = std::stoi(argv[5]);
		gridSize = std::stod(argv[6]);
    }
    else if (methodStr == "FIDUCIAL") {
        method = FIDUCIAL;
        if (argc != 5) {
            std::cerr << "Invalid number of arguments" << std::endl;
            return false;
        }
        markerSize = std::stod(argv[4]);
    }
    else if (methodStr == "IMAGE") {
		method = IMAGE;
        if (argc != 6) {
            std::cerr << "Invalid number of arguments" << std::endl;
			return false;
		}
		imageFile = argv[4];
		imageWidth = std::stod(argv[5]);
    }
    else if (methodStr == "IMPROVED") {
        method = IMPROVED;
        if (argc != 6) {
            std::cerr << "Invalid number of arguments" << std::endl;
            return false;
        }
        imageFile = argv[4];
        imageWidth = std::stod(argv[5]);
    }
    else if (methodStr == "ORB") {
        method = ORB;
        if (argc != 6) {
            std::cerr << "Invalid number of arguments" << std::endl;
            return false;
        }
        imageFile = argv[4];
        imageWidth = std::stod(argv[5]);
    }
    else {
		std::cerr << "Invalid method " << methodStr << std::endl;
		return false;
	}

    return true;
}

void Config::printUsage(std::ostream& os) const {
    os << "SimpleSLAM usage: " << std::endl;
    os << "SimpleSLAM <capture source> <calibration file> <method> <method options...>" << std::endl;
    os << std::endl;
    os << "- capture source: 0 for built-in camera, 1 for USB camera, or a string for a video file" << std::endl;
    os << "- calibration file: file containing camera calibration parameters" << std::endl;
    os << "- method options: depends on method:" << std::endl;
    os << "  - CHECKERBOARD: <grid width> <grid height> <grid size>" << std::endl; 
    os << "    - grid width: number of inner corners across the grid" << std::endl;
    os << "    - grid height: number of inner corners down the grid" << std::endl;
    os << "    - grid size: size of the grid squares in centimetres" << std::endl;
    os << "  - FIDUCIAL: <marker size>" << std::endl;
    os << "    - marker size: size of markers in centimetres" << std::endl;
    os << "  - IMAGE: <image file> <image width>" << std::endl;
    os << "    - image file: file containing the image to track" << std::endl;
    os << "    - image width: width of the image in centimetres" << std::endl;
    os << "  - ORB: <image file> <image width>" << std::endl;
    os << "    - image file: file containing the image to track" << std::endl;
    os << "    - image width: width of the image in centimetres" << std::endl;
    os << "  - IMPROVED: <image file> <image width>" << std::endl;
    os << "    - image file: file containing the image to track" << std::endl;
    os << "    - image width: width of the image in centimetres" << std::endl;
    os << std::endl;
    os << "Examples:" << std::endl;
    os << "SimpleSLAM 0 calibration.txt CHECKERBOARD 14 7 2.0" << std::endl;
    os << "SimpleSLAM 0 calibration.txt FIDUCIAL 20.0" << std::endl;
    os << "SimpleSLAM testFile.mp4 calibration.txt IMAGE image.jpg 10.0" << std::endl;
}