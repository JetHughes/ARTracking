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

	//imshow("image1", image1);
	//imshow("image2", image2);
	//waitKey(0);


	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;


	//detector->detectAndCompute(image1, noArray(), keypoints1, descriptors1);
	detector->detectAndCompute(image2, noArray(), keypoints2, descriptors2);

	vector<vector<DMatch>> matches;
	matcher->knnMatch(descriptors2, matches, 2);

	vector<DMatch> good_matches;
	vector<Point2f> goodpoints2D;
	vector<Point3f> goodpoints3D;

	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i][0].distance < 0.4 * matches[i][1].distance)
		{
			good_matches.push_back(matches[i][0]);
			goodpoints2D.push_back(keypoints2[matches[i][0].queryIdx].pt);
			goodpoints3D.push_back(objectPoints[matches[i][0].trainIdx]);
		}
	}

	std:cout << "good matches: " << good_matches.size() << endl;
	if (good_matches.size() > 20)
	{
		//Mat img_matches;
		//drawMatches(image1, keypoints1, image2, keypoints2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//imshow("Good Matches", img_matches); 
		//waitKey(0);
		if (solvePnPRansac(goodpoints3D, goodpoints2D, camera.K, camera.d, pose.rvec, pose.tvec)) {
			pose.valid = true;
		}
	}
	return pose;





	/*Mat descriptors;
	vector<KeyPoint> keypoints;
	Ptr<SIFT> sift = SIFT::create();
	sift->detectAndCompute(img, noArray(), keypoints, descriptors);

	vector<vector<DMatch>> matches;
	vector<Point2f> imagePoints;

	matcher->knnMatch(descriptors, matches, 2);

	for (const auto& match : matches) {
		if (match[0].distance < 0.8 * match[1].distance) {
			imagePoints.push_back(keypoints[match[0].queryIdx].pt);
		}
	}

	cout << "found " << imagePoints.size() << " matches" << endl;

	if (imagePoints.size() > 50) {
		if (solvePnP(objectPoints, imagePoints, camera.K, camera.d, pose.rvec, pose.tvec)) {
			pose.valid = true;
		}
	}

	return pose;*/
}
