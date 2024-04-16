#include "ImagePoseEstimator.h"

using namespace std;
using namespace cv;

ImagePoseEstimator::ImagePoseEstimator(const Camera& camera, string imageFile, double imageWidth) : 
PoseEstimator(camera), imageWidth(imageWidth) {

	image = imread(imageFile);

	if (image.empty())
	{
		throw runtime_error("Image not found");
	}

	Mat descriptors;
	vector<KeyPoint> keypoints;
	detector = SIFT::create();
	detector->detectAndCompute(image, noArray(), keypoints, descriptors);	

	for (const auto& keypoint : keypoints) {
		objectPoints.push_back(Point3f(keypoint.pt.x, keypoint.pt.y, 0.0));
	}

	matcher = DescriptorMatcher::create("FlannBased");
	matcher->add(descriptors);
}

Pose ImagePoseEstimator::estimatePose(const Mat& img) {
	Pose pose;
	pose.valid = false;

	Mat image1 = image;
	Mat image2 = img;

	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	detector->detectAndCompute(image2, noArray(), keypoints2, descriptors2);

	vector<vector<DMatch>> matches;
	matcher->knnMatch(descriptors2, matches, 2);

	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;

	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i][0].distance < 0.4 * matches[i][1].distance)
		{
			goodpoints2D.push_back(keypoints2[matches[i][0].queryIdx].pt);
			goodpoints3D.push_back(objectPoints[matches[i][0].trainIdx]);
		}
	}

	std:cout << "good matches: " << goodpoints2D.size() << endl;
	if (goodpoints2D.size() > 20)
	{
		if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) {
			pose.valid = true;
		}
	}
	return pose;
}
