#include "ORBPoseEstimator.h"
#include <opencv2/opencv.hpp>
#include "Util.h"

using namespace std;
using namespace cv;

ORBPoseEstimator::ORBPoseEstimator(const Camera& camera, string imageFile, double imageWidth) :
	PoseEstimator(camera), imageWidth(imageWidth) {

	// Read the image
	image = imread(imageFile);
	if (image.empty())
	{
		throw runtime_error("Image not found");
		exit(1);
	}
	cv::Mat grey;
	cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

	// Detect keypoints and compute descriptors
	Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = ORB::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);

	// Create the 3D points for the object
	for (const auto& keypoint : keypoints)
	{
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	// Initialize the matcher with the descriptors
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->add(descriptors);
}

Pose ORBPoseEstimator::estimatePose(const Mat& img) {
	//Init pose
	Pose pose;
	pose.valid = false;

	cv::Mat grey;
	cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);

	// Detect keypoints and compute descriptors
	vector<KeyPoint>keypoints2;
	Mat descriptors2;
	detector->detectAndCompute(grey, noArray(), keypoints2, descriptors2);

	// Match the descriptors
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
	if (goodpoints2D.size() > 20)
	{
		pose = Util::solvePose(goodpoints2D, goodpoints3D, camera, 100);
		if (pose.valid && pose.err > 10) {
			pose = refinePose(pose, goodpoints2D, goodpoints3D);
		}
	}
	return pose;
}

Pose ORBPoseEstimator::refinePose(Pose& pose, vector<Point2f>& goodpoints2D, vector<Point3f>& goodpoints3D) {
	for (size_t i = 0; i < 5; i++)
	{
		// randomly select 20 point matches
		vector<Point2f> randpoints2d;
		vector<Point3f> randpoints3d;
		int idx = rand() % (goodpoints2D.size() - 20);
		for (int i = idx; i < idx+20; i++)
		{
			randpoints2d.push_back(goodpoints2D[i]);
			randpoints3d.push_back(goodpoints3D[i]);
		}

		// compute pose using 8 point matches
		Pose tempPose = Util::solvePose(randpoints2d, randpoints3d, camera);

		// update the best pose
		if (tempPose.valid && tempPose.err < pose.err)
		{
			pose = tempPose;
		}
	}

	return pose;
}

