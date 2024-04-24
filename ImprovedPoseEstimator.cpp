#include "ImprovedPoseEstimator.h"
#include <opencv2/opencv.hpp>
#include "Util.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

ImprovedPoseEstimator::ImprovedPoseEstimator(const Camera& camera, string imageFile, double imageWidth, cv::Mat frame) :
	PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);

	if (image.empty())
	{
		throw runtime_error("image not found");
		exit(1);
	}

	// Detect keypoints and compute descriptors
	cv::Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = SIFT::create();
	detectorH = SIFT::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);
	
	// Initialize the matcher with the descriptors
	matcher = DescriptorMatcher::create("FlannBased");
	matcher->add(descriptors);

	// Create the 3D points for the object
	for (const auto& keypoint : keypoints)
	{
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	// Init separate H (homography) matcher
	matcherH = DescriptorMatcher::create("FlannBased");
	mask = Mat::zeros(frame.size(), frame.type());

}

Pose ImprovedPoseEstimator::estimatePose(const Mat& img) {
	Pose pose;

	// Detect keypoints and compute descriptors
	vector<KeyPoint> keypoints;
	Mat descriptors;

	// If we have a previous frame, track the object using relative pose estimation
	if (!prevFrame.empty() && prevPose.valid)
	{
		//pose = relativePoseEstimation(keypoints, descriptors);
		pose = opticalFlowTracking(img);
	}
	else // Otherwise do absolute pose estimation
	{
		detector->detectAndCompute(img, noArray(), keypoints, descriptors);
		prevFrameKeypoints = keypoints;
		prevFrameDescriptors = descriptors;
		pose = fullDetection(keypoints, descriptors);
		if (pose.valid) {
			cv::Mat grey;
			cvtColor(img, grey, COLOR_BGR2GRAY);
			vector<Point2f> goodFeatures;
			goodFeaturesToTrack(grey, goodFeatures, 100, 0.3, 7, noArray(), 7, false, 0.04);
			prevFrameImagePoints = goodFeatures;
			prevFrame = img;
		}
	}
	prevPose = pose;
	return pose;
}

Pose ImprovedPoseEstimator::opticalFlowTracking(const Mat& img) {
	Pose pose;
	cv::Mat frame_gray;
	cvtColor(img, frame_gray, COLOR_BGR2GRAY);

	cv::Mat prev_grey;
	cvtColor(prevFrame, prev_grey, COLOR_BGR2GRAY);

	// Stuff for drawing images
	vector<Scalar> colors;
	RNG rng;
	for (int i = 0; i < 100; i++)
	{
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}

	// calculate optical flow
	vector<uchar> status;
	vector<float> err;
	vector<Point2f> imagePoints;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(prev_grey, frame_gray, prevFrameImagePoints, imagePoints, status, err, Size(15, 15), 2, criteria);

	vector<Point2f> good_new;
	vector<Point2f> good_old;
	for (uint i = 0; i < prevFrameImagePoints.size(); i++)
	{
		// Select good points
		if (status[i] == 1) {
			good_new.push_back(imagePoints[i]);
			good_old.push_back(prevFrameImagePoints[i]);
			line(mask, imagePoints[i], prevFrameImagePoints[i], colors[i], 2);
			circle(img, imagePoints[i], 5, colors[i], -1);
		}
	}

	cv::Mat flowImg;
	add(img, mask, flowImg);
	imshow("flow", flowImg);

	prevFrameImagePoints = good_new;
	prevFrame = img.clone();

	std::cout << good_new.size() << "\t";

	// estimate pose using homography chaining
	if (good_new.size() > 50) 
	{
		Mat H = findHomography(good_old, good_new, RANSAC);

		// apply H to prevHomography matrix to get the new homography
		Mat newHomography = H * prevHomography;
		prevHomography = newHomography;

		vector<Point2f> projectedRefImagePoints;
		perspectiveTransform(refFrameImagePoints, projectedRefImagePoints, newHomography);

		pose = Util::solvePose(projectedRefImagePoints, refFrameObjectPoints, camera);
	}

	return pose;
}

Pose ImprovedPoseEstimator::relativePoseEstimation(vector<KeyPoint> keypoints, Mat descriptors) {
	// Init pose
	Pose pose;
	pose.valid = false;

	// Find feature matches between the previous frame and the current frame
	vector<vector<DMatch>> matches;
	//detector->detectAndCompute(img, noArray(), keypoints, descriptors);
	matcherH->knnMatch(prevFrameDescriptors, descriptors, matches, 2);

	// Filter the matches
	const float thres = 0.7; 
	vector<Point2f> goodpoints1, goodpoints2;
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i][0].distance < thres * matches[i][1].distance)
		{
			goodpoints1.push_back(prevFrameKeypoints[matches[i][0].queryIdx].pt);
			goodpoints2.push_back(keypoints[matches[i][0].trainIdx].pt);
		}
	}

	if (goodpoints1.size() > 8)
	{
		Mat H = findHomography(goodpoints1, goodpoints2, RANSAC);
		
		// apply H to prevHomography matrix to get the new homography
		Mat newHomography = H * prevHomography;
		prevHomography = newHomography;

		vector<Point2f> projectedRefImagePoints;
		perspectiveTransform(refFrameImagePoints, projectedRefImagePoints, newHomography);

		pose = Util::solvePose(projectedRefImagePoints, refFrameObjectPoints, camera);
	}

	return pose;
}

Pose ImprovedPoseEstimator::fullDetection(vector<KeyPoint> keypoints, Mat descriptors) {
	//Init pose
	Pose pose;
	pose.valid = false;

	// Match the descriptors
	vector<std::vector<DMatch>> matches;
	matcher->knnMatch(descriptors, matches, 2);

	// Filter the matches
	// Get the 2D and 3D points for the good matches
	double threshold = 0.7;
	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i][0].distance < threshold * matches[i][1].distance)
		{
			goodpoints2D.push_back(keypoints[matches[i][0].queryIdx].pt);
			goodpoints3D.push_back(objectPoints[matches[i][0].trainIdx] * imageWidth / image.cols);
		}
	}
	std::cout << goodpoints2D.size() << "\t";

	// Estimate the pose if we have at least 20 good matches
	if (goodpoints2D.size() > 20) {
		if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec))
		{
			double reprojectionError = Util::computeReprojectionError(goodpoints3D, goodpoints2D, pose.rvec, pose.tvec, camera.K, camera.d);
			if (reprojectionError < 10)
			{
				pose.valid = true;
				refFrameObjectPoints = goodpoints3D;
				refFrameImagePoints = goodpoints2D;
				prevHomography = Mat::eye(3, 3, CV_64F);
				//std::cout << "abs";
				//pose.toString();
			}
			pose.err = reprojectionError;
		}
	}

	//if (!pose.valid && prevPose.valid)
	//{
	//	// If we can't find the pose and we had a reasonable estimation in the previous frame, 
	//	// try to track the object using the previous frame
	//	pose = relativePoseEstimation(keypoints, descriptors);
	//	std::cout << "r";
	//}

	return pose;
}