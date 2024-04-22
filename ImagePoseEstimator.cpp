#include "ImagePoseEstimator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Compute the reprojection error
// https://chat.openai.com/share/21126c54-3fa6-4017-a63e-3f0ea34728a7
double computeReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::InputArray distortionCoeffs) {
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

ImagePoseEstimator::ImagePoseEstimator(const Camera& camera, string imageFile, double imageWidth) : 
PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);
	if (image.empty())
	{
		throw runtime_error("Image not found");
		exit(1);
	}
	//cv::imshow("Image", image);
	//cv::waitKey(0);
	
	// Detect keypoints and compute descriptors
	cv::Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = SIFT::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);	

	// Create the 3D points for the object
	for (const auto& keypoint : keypoints)
	{
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	// Initialize the matcher with the descriptors
	matcher = DescriptorMatcher::create("FlannBased");
	matcher->add(descriptors);
}

Pose ImagePoseEstimator::estimatePose(const Mat& img) {
	//Init pose
	Pose pose;
	pose.valid = false;

	// Detect keypoints and compute descriptors
	vector<KeyPoint>keypoints2;
	Mat descriptors2;
	detector->detectAndCompute(img, noArray(), keypoints2, descriptors2);

	// Match the descriptors
	vector<std::vector<DMatch>> matches;
	matcher->knnMatch(descriptors2, matches, 2);

	// Filter the matches
	// Get the 2D and 3D points for the good matches
	double threshold = 0.7;
	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i][0].distance < threshold * matches[i][1].distance)
		{
			goodpoints2D.push_back(keypoints2[matches[i][0].queryIdx].pt);
			goodpoints3D.push_back(objectPoints[matches[i][0].trainIdx] * imageWidth / image.cols);
		}
	}
	std::cout << goodpoints2D.size() << "\t";

	// Estimate the pose if we have at least 20 good matches
	if (goodpoints2D.size() > 20)
	{
		if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) 
		{
			double reprojectionError = computeReprojectionError(goodpoints3D, goodpoints2D, pose.rvec, pose.tvec, camera.K, camera.d);
			if (reprojectionError < 100) 
			{ 
				pose.valid = true;
				//pose.toString();
			}
			pose.err = reprojectionError;
		}
	}
	return pose;
}
