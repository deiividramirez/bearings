#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int compute_descriptors(const Mat &img,
								const vc_parameters &params,
								const vc_desired_configuration &desired_configuration,
								vc_homograpy_matching_result &result)
{

	/*** KP ***/
	Mat descriptors;
	vector<KeyPoint> kp; // kp and descriptors for current image

	/*** Creatring ORB object ***/
	Ptr<ORB> orb = ORB::create(params.nfeatures, params.scaleFactor, params.nlevels, params.edgeThreshold, params.firstLevel, params.WTA_K, params.scoreType, params.patchSize, params.fastThreshold);
	orb->detect(img, kp);

	if (kp.size() == 0)
	{
		cout << "[ERROR] No keypoints detected" << endl;
		return -1;
	}
	else
	{
		cout << "[INFO] Keypoints detected: " << kp.size() << endl;
	}
	orb->compute(img, kp, descriptors);

	/************************************************************* Using flann for matching*/
	FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));
	vector<vector<DMatch>> matches;
	matcher.knnMatch(desired_configuration.descriptors, descriptors, matches, 2);

	/************************************************************* Processing to get only goodmatches*/

	vector<DMatch> goodMatches;
	for (int i = 0; i < matches.size(); ++i)
	{
		if (matches[i][0].distance < matches[i][1].distance * params.flann_ratio)
			goodMatches.push_back(matches[i][0]);
	}
	if (goodMatches.size() == 0)
	{
		cout << "[ERROR] There are no good matches" << endl;
		exit(-1);
	}

	/************************************************************* Getting descriptors */

	//  TODO comparar con compute homography
	//-- transforming goodmatches to points
	result.p1.release();
	result.p2.release();
	result.p1 = Mat(goodMatches.size(), 2, CV_64F);
	result.p2 = Mat(goodMatches.size(), 2, CV_64F);
	// cout << "n good matches " << goodMatches.size() << endl;
	for (int i = 0; i < goodMatches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		// 		result.p1.push_back(Mat(desired_configuration.kp[goodMatches[i].queryIdx].pt));
		// 		result.p2.push_back(Mat(kp[goodMatches[i].trainIdx].pt));
		int idx = goodMatches[i].queryIdx;

		Mat tmp = Mat(desired_configuration.kp[idx].pt).t();
		tmp.copyTo(result.p1.row(i));
		tmp.release();
		idx = goodMatches[i].trainIdx;
		tmp = Mat(kp[idx].pt).t();
		tmp.copyTo(result.p2.row(i));
		// cout << "good match " << result.p1.row(i) << " <-> " << result.p2.row(i) << endl;
	}
	//     result.p1 = result.p1.reshape(goodMatches.size(),2);
	//     result.p2 = result.p2.reshape(goodMatches.size(),2);
	// computing error

	Mat a = Mat(result.p1);
	Mat b = Mat(result.p2);
	result.mean_feature_error = norm(a, b) / ((double)result.p1.rows);

	cout << "[INFO] Mean feature error: " << result.mean_feature_error << endl;

	// cout << "Result p1: " << result.p1 << endl;
	// cout << "Result p2: " << result.p2 << endl;
	// // 752 x 480
	// exit(-1);
	// // circle the keypoints in the image
	// for (int i = 0; i < result.p1.rows; i++)
	// {
	// 	circle(img, Point(result.p1.at<double>(i, 0), result.p1.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
	// }

	// for (int i = 0; i < result.p2.rows; i++)
	// {
	// 	circle(desired_configuration.img, Point(result.p2.at<double>(i, 0), result.p2.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
	// }

	// // show the image
	// imshow("ORB", img);
	// imshow("ORB desired", desired_configuration.img);
	// waitKey(0);

	// exit(-1);

	// Finding homography
	// 	result.H = findHomography(result.p1, result.p2 ,RANSAC, 0.5);
	// 	if (result.H.rows==0)
	// 		return -1;
	/************************************************************* Draw matches */

	result.img_matches = Mat::zeros(img.rows, img.cols * 2, img.type());
	drawMatches(desired_configuration.img, desired_configuration.kp, img, kp,
					goodMatches, result.img_matches,
					Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	return 0;
}