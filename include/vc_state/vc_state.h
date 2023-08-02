#ifndef VC_STATE_H
#define VC_STATE_H

#define RESET_C "\033[0m"
#define BLACK_C "\033[30m"              /* Black */
#define RED_C "\033[31m"                /* Red */
#define GREEN_C "\033[32m"              /* Green */
#define YELLOW_C "\033[33m"             /* Yellow */
#define BLUE_C "\033[34m"               /* Blue */
#define MAGENTA_C "\033[35m"            /* Magenta */
#define CYAN_C "\033[36m"               /* Cyan */
#define WHITE_C "\033[37m"              /* White */
#define BOLDBLACK_C "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED_C "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN_C "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW_C "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE_C "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA_C "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN_C "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE_C "\033[1m\033[37m"   /* Bold White */
#define M_PI 3.14159265358979323846     /* pi */

#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

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

    int control = 1, camara = 1;

    Mat seguimiento;
    Mat bearing;

    cv::Mat K;
    cv::Mat Kinv;

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

/****************** STRUCT FOR POINTS IN THE IMAGES ******************/
typedef struct savingData
{
    Mat img;
    Mat imgGray;

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners;
    Mat points;
    Mat normPoints;

    Mat inSphere;
    Mat bearings;

} savingData;

/****************** STRUCT FOR THE DESIRED CONFIGURATION ******************/
typedef struct vc_desired_configuration
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;
    cv::Mat img;
} vc_desired_configuration;

class vc_state
{
public:
    /* defining where the drone will move and integrating system */
    float X = 0.0, Y = 0.0, Z = 0.0, Yaw = 0.0, Pitch = 0.0, Roll = 0.0;
    bool initialized = false;

    /* Control parameters  */
    float Vx = 0.0, Vy = 0.0, Vz = 0.0;
    float Vyaw = 0.0, Vroll = 0.0, Vpitch = 0.0;

    float Kv = 1.0, Kw = 1.0, Kv_i = 1.0;
    float Kv_max = 1.0, Kw_max = 1.0, Kv_i_max = 1.0;
    float kv_prima = 1.0, kw_prima = 1.0, kv_i_prima = 1.0;
    float lambda_kvp = 0, lambda_kvi = 0, lambda_kw = 0;

    float dt = 0.025;
    float t = 0;

    float error = 0;
    float error_pix = 0;

    Mat I3 = Mat::eye(3, 3, CV_64F);

    Mat R;
    Mat Q;
    vector<Mat> Qi;

    vector<Mat> Hi;

    Mat U_trans = Mat::zeros(3, 1, CV_64F);
    Mat U_rot = Mat::zeros(3, 1, CV_64F);

    Mat integral_error = Mat::zeros(3, 1, CV_64F);
    Mat integral_error6 = Mat::zeros(6, 1, CV_64F);
    Mat integral_error12 = Mat::zeros(12, 1, CV_64F);

    bool in_target = false;

    vc_parameters params;
    //  Desired configuration
    // vc_desired_configuration desired_configuration;

    savingData desired;
    savingData actual;

    Mat groundTruth = Mat::zeros(6, 1, CV_64F);
    //  Best approximations
    // bool selected = false;
    // cv::Mat t_best;
    // cv::Mat R_best; // to save the rot and trans

    /* ROS pub */
    // ros::Publisher ros_pub;

    // Methods
    vc_state();

    std::pair<Eigen::VectorXd, float> update();
    void load(const ros::NodeHandle &nh);
    void initialize(const float &x,
                    const float &y,
                    const float &z,
                    const float &yaw);
};

// Create the Struct which be able to
// store the information about the points
typedef struct vecDist
{
    int i;        // index i
    int j;        // index j p_i * p_j
    double dist;  // distance between p2_i and p2_j
    double dist2; // distance between p1_i and p1_j
} vecDist;

/****************** FUNCTIONS TO USE ******************/

/****************** FUNCTIONS TO USE ******************/

/****************** SAVE DESIRED IMAGES FROM POSES ******************/
void saveDesired1f(const sensor_msgs::Image::ConstPtr &msg);
void saveDesired2f(const sensor_msgs::Image::ConstPtr &msg);
void saveDesired3f(const sensor_msgs::Image::ConstPtr &msg);
void saveDesired4f(const sensor_msgs::Image::ConstPtr &msg);
void saveDesired5f(const sensor_msgs::Image::ConstPtr &msg);

// /****************** COMPUTE DESCRIPTORS FOR IMAGES ******************/
// int compute_descriptors(const cv::Mat &img,
//                         const vc_parameters &params,
//                         const vc_desired_configuration &desired_configuration,
//                         vc_homograpy_matching_result &result);

/****************** GET MOORE-PENROSE PSEUDO-INVERSE OF A MATRIX ******************/
cv::Mat Moore_Penrose_PInv(cv::Mat L, double &det);

/****************** TRAJECTORY TRACKER KLT - ALGORITHM ******************/
// int Compute_descriptors(const Mat &actual,
//                         Mat &img_points,
//                         vc_state &state,
//                         vc_homograpy_matching_result &matching_result);

/****************** ARUCO DETECTOR FROM OPENCV 4+ ******************/
int aruco_detector(const Mat &actual,
                   Mat &img_points,
                   vc_state &state,
                   vc_homograpy_matching_result &matching_result,
                   XmlRpc::XmlRpcValue marker_idXLM);

int Kanade_Lucas_Tomasi(const Mat &img_old,
                        const Mat &img_new,
                        Mat &desired_temp,
                        Mat &img_points,
                        vc_state &state,
                        vc_homograpy_matching_result &matching_result);

int getBearing(Mat &actual_image,
               Mat &store_bearing,
               Mat &store_ground_truth,
               vc_state *state,
               int drone_id,
               vector<geometry_msgs::PoseStamped> &pos_dron);

Mat signMat(Mat mat);
Mat robust(Mat error);

Mat composeR(double roll, double pitch, double yaw);

Mat composeR(Mat rvec);

Mat decomposeR(Mat R);

Mat projOrtog(Mat &x);

Mat puntoMedio(Mat p1, Mat p2, Mat p3, Mat p4);
Mat puntoMedio(Mat p1, Mat p2);

string type2str(int type);

void Tipito(Mat &Matrix);

void clip(Mat &Matrix, int max = 5, int min = -5);
void clip(double value, int max = 5, int min = -5);

#endif
