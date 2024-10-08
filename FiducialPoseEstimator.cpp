#include "FiducialPoseEstimator.h"

using namespace std;
using namespace cv;

FiducialPoseEstimator::FiducialPoseEstimator(const Camera& camera):
	PoseEstimator(camera) {
	// Create a dictionary of markers
	detector = cv::aruco::ArucoDetector(dictionary, detectorParams);

	// Create marker corner object points
	for (int i = 0; i < markerIds.size(); i++) {
		std::vector<cv::Point3f> markerObjPoints;
		markerObjPoints.push_back(cv::Point3f(markerLocations[i].x, markerLocations[i].y, 0));
		markerObjPoints.push_back(cv::Point3f(markerLocations[i].x + markerLength, markerLocations[i].y, 0));
		markerObjPoints.push_back(cv::Point3f(markerLocations[i].x, markerLocations[i].y + markerLength, 0));
		markerObjPoints.push_back(cv::Point3f(markerLocations[i].x + markerLength, markerLocations[i].y + markerLength, 0));
		objPoints.push_back(markerObjPoints);
	}

	// Create board object
	board = cv::aruco::Board(objPoints, dictionary, markerIds);
}

Pose FiducialPoseEstimator::estimatePose(const Mat& img) {
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	//Init pose
	Pose pose;
	pose.valid = false;

	// Detect markers
	std::vector<int> foundMarkerIds;
	std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
	detector.detectMarkers(img, corners, foundMarkerIds, rejectedCandidates);
	detector.refineDetectedMarkers(img, board, corners, foundMarkerIds, rejectedCandidates);
	std::cout << foundMarkerIds.size() << ",";

	// If at least one marker detected
	if (foundMarkerIds.size() > 0) {
		int nMarkers = corners.size();
		std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
		// Calculate pose for each marker
		std::vector<cv::Point3f> tempObjPoints;
		tempObjPoints.push_back(cv::Point3f(0, 0, 0));
		tempObjPoints.push_back(cv::Point3f(markerLength, 0, 0));
		tempObjPoints.push_back(cv::Point3f(markerLength, markerLength, 0));
		tempObjPoints.push_back(cv::Point3f(0, markerLength, 0));

		for (int i = 0; i < foundMarkerIds.size(); i++) {
			solvePnP(tempObjPoints, corners.at(i), camera.K, camera.d, rvecs.at(i), tvecs.at(i));
		}

		// Draw axis for each marker
		for (unsigned int i = 0; i < foundMarkerIds.size(); i++) {
			cv::aruco::drawDetectedMarkers(img, corners, foundMarkerIds);
			cv::drawFrameAxes(img, camera.K, camera.d, rvecs[i], tvecs[i], 1);
		}
		
		// Get object and image points for the solvePnP function
		std::vector<cv::Point3f> outObjPoints;
		std::vector<cv::Point2f> imgPoints;
		board.matchImagePoints(corners, foundMarkerIds, outObjPoints, imgPoints);

		// Find pose
		cv::solvePnP(outObjPoints, imgPoints, camera.K, camera.d, pose.rvec, pose.tvec);
 		pose.valid = true;
	}
	return pose;
}


