#include "ImprovedPoseEstimator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

ImprovedPoseEstimator::ImprovedPoseEstimator(const Camera& camera, string imageFile, double imageWidth) :
	PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);
	if (image.empty())
	{
		throw runtime_error("Image not found");
	}
	//cv::imshow("Image", image);
	//cv::waitKey(0);

	// Detect keypoints and compute descriptors
	cv::Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = ORB::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);

	// Create the 3D points for the object
	for (const auto& keypoint : keypoints) {
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	// Initialize the matcher with the descriptors
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->add(descriptors);
}

Pose ImprovedPoseEstimator::estimatePose(const Mat& img) {
	Pose pose;
	pose.valid = false;

	// Detect keypoints and compute descriptors
	vector<KeyPoint>keypoints2;
	Mat descriptors2;

	detector->detectAndCompute(img, noArray(), keypoints2, descriptors2);

	// Match the descriptors
	vector<DMatch> matches;
	matcher->match(descriptors2, matches);

	// keep only the best 20% of matches
	std::sort(matches.begin(), matches.end());
	const int numGoodMatches = matches.size() * 0.2;
	matches.erase(matches.begin() + numGoodMatches, matches.end());

	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;

	// Get the 2D and 3D points for the good matches
	for (int i = 0; i < matches.size(); i++)
	{
		goodpoints2D.push_back(keypoints2[matches[i].queryIdx].pt);
		goodpoints3D.push_back(objectPoints[matches[i].trainIdx] * imageWidth / image.cols);
	}

std:cout << "good matches: " << goodpoints2D.size() << endl;

	// Estimate the pose if we have at least 20 good matches
	if (goodpoints2D.size() > 20)
	{
		if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) {
			pose.valid = true;
			pose.toString();
		}
	}
	return pose;
}

Pose ImprovedPoseEstimator::estimatePoseHomo(const Mat& img) {
	Pose pose;
	pose.valid = false;

	// detect features
	cv::Ptr<cv::FeatureDetector> detector = SIFT::create();
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	detector->detectAndCompute(img, cv::noArray(), keypoints1, descriptors1);
	detector->detectAndCompute(prevFrame, cv::noArray(), keypoints2, descriptors2);

	// match features
	Ptr<DescriptorMatcher> matcher = cv::FlannBasedMatcher::create();
	vector<vector<DMatch>> matches;
	matcher->knnMatch(descriptors1, descriptors2, matches, 2);

	// filter matches, and create 2d and 3d points
	vector<DMatch> goodMatches;
	vector<Point2f> goodPoints2D;
	vector<Point3f> goodPoints3D;
	for (const auto& match : matches) {
		if (match[0].distance < 0.8 * match[1].distance) {
			goodMatches.push_back(match[0]);
			// is there a way to optimie the object points using the previously known pose??
			goodPoints2D.push_back(keypoints1[match[0].trainIdx].pt);
			goodPoints3D.push_back(Point3f(keypoints2[match[0].queryIdx].pt.x, keypoints2[match[0].queryIdx].pt.y, 0));
		}
	}




}
