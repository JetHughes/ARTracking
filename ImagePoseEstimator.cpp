#include "ImagePoseEstimator.h"
#include <opencv2/opencv.hpp>
#include "Util.h"

using namespace std;
using namespace cv;

ImagePoseEstimator::ImagePoseEstimator(const Camera& camera, string imageFile, double imageWidth) : 
PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);
	if (image.empty())
	{
		throw runtime_error("Image not found");
		exit(1);
	}
	
	// Detect keypoints and compute descriptors
	Mat descriptors;
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
	vector<vector<DMatch>> matches;
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
	cout << goodpoints2D.size() << ",";

	// Estimate the pose if we have at least 20 good matches
	if (goodpoints2D.size() > 20)
	{
		pose = Util::solvePose(goodpoints2D, goodpoints3D, camera);
	}
	return pose;
}
