#include "ImprovedPoseEstimator.h"
#include <opencv2/opencv.hpp>
#include "Util.h"
#include <iostream>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

ImprovedPoseEstimator::ImprovedPoseEstimator(const Camera& camera, string imageFile, double imageWidth) :
	PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);

	if (image.empty())
	{
		throw runtime_error("image not found");
		exit(1);
	}

	// Detect keypoints and compute descriptors
	Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = ORB::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);
	
	// Initialize the matcher with the descriptors
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->add(descriptors);

	// Create the 3D points for the object
	for (const auto& keypoint : keypoints)
	{
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	// Generate random colours for the optical flow drawing
	RNG rng;
	for (int i = 0; i < 100; i++)
	{
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}
}

Pose ImprovedPoseEstimator::estimatePose(const Mat& img) {
	Pose pose;
	pose.valid = false;

	// If we have a valid pose, track the object using relative pose estimation
	if (prevPose.valid)
	{
		pose = opticalFlowTracking(img);
	}
	else // Otherwise do absolute pose estimation
	{
		pose = getInitialPose(img);

		// If we find a valid pose initialise the optical flow tracking
		if (pose.valid) {
			Mat frameGrey;
			cvtColor(img, frameGrey, COLOR_BGR2GRAY);
			goodFeaturesToTrack(frameGrey, trackingFeatures, 100, 0.3, 7, noArray(), 7, false, 0.04);
			prevFrame = frameGrey;
			mask = Mat::zeros(img.size(), img.type()); // mask for drawing optical flow
		}
	}
	prevPose = pose;
	return pose;
}

// adapted from https://github.com/opencv/opencv/blob/4.x/samples/cpp/tutorial_code/video/optical_flow/optical_flow.cpp
Pose ImprovedPoseEstimator::opticalFlowTracking(const Mat& img) {
	Mat frameGrey;
	cvtColor(img, frameGrey, COLOR_BGR2GRAY);

	Pose pose;
	pose.valid = false;

	// calculate optical flow
	vector<uchar> status;
	vector<float> err;
	vector<Point2f> imageFeatures;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(prevFrame, frameGrey, trackingFeatures, imageFeatures, status, err, Size(15, 15), 2, criteria);

	// Select good points
	vector<Point2f> goodPointsNew;
	vector<Point2f> goodPointsOld;
	for (uint i = 0; i < trackingFeatures.size(); i++)
	{
		if (status[i] == 1) {
			goodPointsNew.push_back(imageFeatures[i]);
			goodPointsOld.push_back(trackingFeatures[i]);
			line(mask, imageFeatures[i], trackingFeatures[i], colors[i], 2);
			circle(img, imageFeatures[i], 5, colors[i], -1);
		}
	}
	cout << goodPointsNew.size() << ",";

	// Draw Visualisation
	Mat result;
	add(img, mask, result);
	imshow("Optical Flow Visualisation", result);

	// Update the tracking features and the previous frame
	trackingFeatures = goodPointsNew;
	prevFrame = frameGrey.clone();

	// Estimate pose using homography chaining
	if (goodPointsNew.size() > 50) 
	{
		// calculate the homography and chain it with the previous homography
		H = findHomography(goodPointsOld, goodPointsNew, RANSAC) * H;

		// project the 2D points of the reference frame to the current frame
		vector<Point2f> projectedRefImagePoints;
		perspectiveTransform(refFrameImagePoints, projectedRefImagePoints, H);

		// solve the pose using the projected 2D points and the 3D points from the reference frame
		pose = Util::solvePose(projectedRefImagePoints, refFrameObjectPoints, camera);
	}

	return pose;
}

Pose ImprovedPoseEstimator::getInitialPose(const Mat& img) {
	//Init pose
	Pose pose;
	pose.valid = false;

	// Detect keypoints and compute descriptors
	vector<KeyPoint> keypoints2;
	Mat descriptors2;
	detector->detectAndCompute(img, noArray(), keypoints2, descriptors2);

	// Match the descriptors2 with the saved descriptors
	vector<DMatch> matches;
	matcher->match(descriptors2, matches);

	// Filter the matches
	// Get the 2D and 3D points for the good matches
	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;
	std::sort(matches.begin(), matches.end());
	const int numGoodMatches = matches.size() * 0.1;
	matches.erase(matches.begin() + numGoodMatches, matches.end());
	for (int i = 0; i < matches.size(); i++)
	{
		goodpoints2D.push_back(keypoints2[matches[i].queryIdx].pt);
		goodpoints3D.push_back(objectPoints[matches[i].trainIdx] * imageWidth / image.cols);
	}
	std::cout << goodpoints2D.size() << ",";

	// Estimate the pose if we have at least 20 good matches
	if (goodpoints2D.size() > 20) {
		pose = Util::solvePose(goodpoints2D, goodpoints3D, camera);

		if (pose.valid)
		{
			refFrameObjectPoints = goodpoints3D;
			refFrameImagePoints = goodpoints2D;
			H = Mat::eye(3, 3, CV_64F);
		}
	}

	return pose;
}