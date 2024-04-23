#include "ImprovedPoseEstimator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Compute the reprojection error
// https://chat.openai.com/share/21126c54-3fa6-4017-a63e-3f0ea34728a7
double ImprovedPoseEstimator::computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs) {
	std::vector<cv::Point2f> projectedPoints;
	projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoeffs, projectedPoints);

	double totalError = 0.0;
	for (size_t i = 0; i < imagePoints.size(); ++i)
	{
		double dx = projectedPoints[i].x - imagePoints[i].x;
		double dy = projectedPoints[i].y - imagePoints[i].y;
		double error = sqrt(dx * dx + dy * dy);
		totalError += error;
	}

	double meanError = totalError / imagePoints.size();
	return meanError;
}

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
}

Pose ImprovedPoseEstimator::estimatePose(const Mat& img) {
	Pose pose;

	// Detect keypoints and compute descriptors
	vector<KeyPoint> keypoints;
	Mat descriptors;
	detector->detectAndCompute(img, noArray(), keypoints, descriptors);

	//// If we have a previous frame, track the object using relative pose estimation
	//if (!prevFrame.empty() && prevPose.valid)
	//{
	//	pose = relativePoseEstimation(keypoints, descriptors);
	//}
	//else // Otherwise do absolute pose estimation
	//{
		pose = fullDetection(keypoints, descriptors);
	//}
	prevFrame = img;
	prevPose = pose;
	prevFrameKeypoints = keypoints;
	prevFrameDescriptors = descriptors;
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
		// Using the essential matrix estimate the relative pose of the new frame
		cv::Mat E, R, t;
		E = findEssentialMat(goodpoints1, goodpoints2, camera.K, RANSAC);
		if (!E.empty())
		{
			int numIniliers = recoverPose(E, goodpoints1, goodpoints2, camera.K, R, t);
			if (numIniliers > 0) {

				// Using the relative pose estimate the new absolute pose
				// Pose ac is a combinatin of pose ab and pose bc
				Mat prevFrameR;
				Rodrigues(prevPose.rvec, prevFrameR);

				Rodrigues(prevFrameR * R, pose.rvec); // Rac = Rab * Rbc
				pose.tvec = prevPose.tvec + prevFrameR * t; // tac = tab + Rab * tbc

				pose.valid = true;
				//std::cout << "rel";
				pose.err = -2;
			}
		}
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
			double reprojectionError = computeReprojectionError(goodpoints3D, goodpoints2D, pose.rvec, pose.tvec, camera.K, camera.d);
			if (reprojectionError < 30)
			{
				pose.valid = true;
				//std::cout << "abs";
				//pose.toString();
			}
			pose.err = reprojectionError;
		}
	}

	if (!pose.valid && prevPose.valid)
	{
		// If we can't find the pose and we had a reasonable estimation in the previous frame, 
		// try to track the object using the previous frame
		pose = relativePoseEstimation(keypoints, descriptors);
		std::cout << "r";
	}

	return pose;
}