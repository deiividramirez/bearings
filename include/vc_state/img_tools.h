#ifndef IMG_TOOLS_H
#define IMG_TOOLS_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>

typedef struct vc_parameters
{

    // Image proessing parameters
    float feature_threshold = 1;
    int nfeatures = 250;
    float scaleFactor = 1.2;
    int nlevels = 8;
    int edgeThreshold = 15; // Changed default (31);
    int firstLevel = 0;
    int WTA_K = 2;
    cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize = 30;
    int fastThreshold = 20;
    float flann_ratio = 0.7;
    int control = 1;
    int camara = 1;
    float gainv = 2.;
    float gainw = 2.;

    // Camera parameters
    cv::Mat K;

} vc_parameters;

typedef struct vc_homograpy_matching_result
{
    cv::Mat H;
    cv::Mat img_matches;
    // std::vector<cv::Point2f> p1;
    // std::vector<cv::Point2f> p2;
    cv::Mat p1;
    cv::Mat p2;
    double mean_feature_error = 1e10;
    double mean_feature_error_pix = 1e10;
} vc_homograpy_matching_result;

typedef struct vc_desired_configuration
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;
    cv::Mat img;
} vc_desired_configuration;

int compute_homography(const cv::Mat &img,
                       const vc_parameters &params,
                       const vc_desired_configuration &desired_configuration,
                       vc_homograpy_matching_result &result);
int select_decomposition(const std::vector<cv::Mat> &Rs,
                         const std::vector<cv::Mat> &Ts,
                         const std::vector<cv::Mat> &Ns,
                         const vc_homograpy_matching_result &matching_result,
                         bool &selected,
                         cv::Mat &Rbest, cv::Mat &tbest);
int compute_descriptors(const cv::Mat &img,
                        const vc_parameters &params,
                        const vc_desired_configuration &desired_configuration,
                        vc_homograpy_matching_result &result);

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R);
void camera_norm(const vc_parameters &params,
                 vc_homograpy_matching_result &result);
cv::Mat interaction_Mat(vc_homograpy_matching_result &result,
                        double Z);
cv::Mat Moore_Penrose_PInv(cv::Mat L, double &det);

#endif
