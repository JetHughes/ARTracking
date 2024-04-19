#include "ImagePoseEstimator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

ImagePoseEstimator::ImagePoseEstimator(const Camera& camera, string imageFile, double imageWidth) : 
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

Pose ImagePoseEstimator::estimatePose(const Mat& img) {
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
		goodpoints3D.push_back(objectPoints[matches[i].trainIdx]*imageWidth/image.cols);
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
