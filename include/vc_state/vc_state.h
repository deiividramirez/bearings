#ifndef IMG_TOOLS_H
#define IMG_TOOLS_H

#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

/****************** PARAMETERS FOR IMAGE PROCESSING ******************/
typedef struct vc_parameters
{
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

/****************** STRUCT FOR POINTS IN THE IMAGES ******************/
typedef struct vc_homograpy_matching_result
{
    cv::Mat H;
    cv::Mat img_matches;
    cv::Mat p1;
    cv::Mat p2;
    double mean_feature_error = 1e10;
    double mean_feature_error_pix = 1e10;
} vc_homograpy_matching_result;

/****************** STRUCT FOR THE DESIRED CONFIGURATION ******************/
typedef struct vc_desired_configuration
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;
    cv::Mat img;
} vc_desired_configuration;

#endif



#ifndef VC_STATE_H
#define VC_STATE_H

#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

class vc_state
{
public:
    /* defining where the drone will move and integrating system*/
    float X = 0.0, Y = 0.0, Z = 0.0, Yaw = 0.0, Pitch = 0.0, Roll = 0.0;
    bool initialized = false;
    // 		float t,dt;
    // 		float Kv,Kw;

    /* Control parameters  */
    float Vx = 0.0, Vy = 0.0, Vz = 0.0;
    float Vyaw = 0.0, Vroll = 0.0, Vpitch = 0.0;
    float Kv = 1.0;
    float Kw = 1.0;
    float dt = 0.025;
    float t = 0;
    float lambda = 0;

    // Image proessing parameters
    vc_parameters params;
    //  Desired configuration
    vc_desired_configuration desired_configuration;

    //  Best approximations
    bool selected = false;
    cv::Mat t_best;
    cv::Mat R_best; // to save the rot and trans

    /* ROS pub */
    ros::Publisher ros_pub;

    // Methods
    vc_state();

    std::pair<Eigen::VectorXd, float> update();
    void load(const ros::NodeHandle &nh);
    void initialize(const float &x,
                    const float &y,
                    const float &z,
                    const float &yaw);
    //         int (*controller )[1];
};

/****************** FUNCTIONS TO USE ******************/

/****************** MAIN CONTROL FOR IMAGE BASED VISUAL SERVOING ******************/
int GUO(cv::Mat img,
        vc_state &state,
        vc_homograpy_matching_result &matching_result);

/****************** FUNCTIONS TO USE ******************/

/****************** COMPUTE DESCRIPTORS FOR IMAGES ******************/
int compute_descriptors(const cv::Mat &img,
                        const vc_parameters &params,
                        const vc_desired_configuration &desired_configuration,
                        vc_homograpy_matching_result &result);

/****************** GET MOORE-PENROSE PSEUDO-INVERSE OF A MATRIX ******************/
cv::Mat Moore_Penrose_PInv(cv::Mat L, double &det);

/****************** TRAJECTORY TRACKER KLT - ALGORITHM ******************/
int Kanade_Lucas_Tomasi(const Mat &actual,
                        Mat &img_points,
                        vc_state &state,
                        vc_homograpy_matching_result &matching_result);

/****************** ARUCO DETECTOR FROM OPENCV 4+ ******************/
int aruco_detector(const Mat &actual,
                   Mat &img_points,
                   vc_state &state,
                   vc_homograpy_matching_result &matching_result);
#endif