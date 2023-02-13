/****************** ROS LIBRARIES ******************/
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mav_msgs/conversions.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

/****************** OPENCV LIBRARIES ******************/
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

/****************** C++ LIBRARIES ******************/
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <iostream>

/****************** CUSTOM LIBRARIES ******************/
#include "vc_state/vc_state.h"

/****************** FUNCTION NAMES ******************/

/****************** FOR WRITING MATRIXES ******************/
void writeFile(vector<float> &vec, const string &name);

/****************** FOR CAMERA CALLBACKS ******************/
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void imageCallback2(const sensor_msgs::Image::ConstPtr &msg);
// void imageCallback3(const sensor_msgs::Image::ConstPtr &msg);
// void imageCallback4(const sensor_msgs::Image::ConstPtr &msg);

/****************** FOR TRAYECTORY CALLBACKS ******************/
void poseCallback1(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback2(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback4(const geometry_msgs::Pose::ConstPtr &msg);

/****************** FOR BEARINGS CALLBACKS ******************/
// void IMGCallback1(const sensor_msgs::Image::ConstPtr &msg);
// void IMGCallback2(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback4(const sensor_msgs::Image::ConstPtr &msg);

/****************** FOR IMU CALLBACKS ******************/
void imuCallback1(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback2(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback3(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback4(const sensor_msgs::Imu::ConstPtr &msg);

/****************** FOR CONVERT FROM YALM TO OPENCV MAT ******************/
Mat convertBearing(XmlRpc::XmlRpcValue bearing, XmlRpc::XmlRpcValue segs);

/****************** SAVE REAL POSITION OF DRONES ******************/
geometry_msgs::PoseStamped pos_dron1, pos_dron2, pos_dron3, pos_dron4;

/****************** ARRAYS WITH DATA FOR EASY USAGE ******************/
// vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2, imageCallback3, imageCallback4};
vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2};
vector<void (*)(const geometry_msgs::Pose::ConstPtr &)> posesCallback = {poseCallback1, poseCallback2, poseCallback3, poseCallback4};
vector<void (*)(const sensor_msgs::Imu::ConstPtr &)> imuCallbacks = {imuCallback1, imuCallback2, imuCallback3, imuCallback4};
vector<geometry_msgs::PoseStamped> pos_dron = {pos_dron1, pos_dron2, pos_dron3, pos_dron4};

/****************** DECLARING OBJECTS TO RECEIVE IMAGE MESSAGES ******************/
sensor_msgs::ImagePtr image_msg;

/****************** WORKSPACE DEFINITION FROM CMAKE ******************/
string workspace = WORKSPACE;

/****************** VISUAL CONTROL STATES AND RESULT VARIABLES ******************/
vc_state state_1, state_2, state_3, state_4;
vc_homograpy_matching_result matching_result_1, matching_result_2, matching_result_3, matching_result_4;

/****************** ARRAYS WITH DATA FOR EASY USAGE ******************/
vector<vc_state> states = {state_1, state_2, state_3, state_4};
vector<vc_homograpy_matching_result> matching_results = {matching_result_1, matching_result_2, matching_result_3, matching_result_4};

/****************** AUXILIAR GLOBAL VARIABLES ******************/

// For saving images and counting them
Mat img_old1, img_points1, img_old2, img_points2, img_old3, img_points3, img_old4, img_points4;
int contIMG1 = 0, contIMG2 = 0, contIMG3 = 0, contIMG4 = 0, contGEN = 0, SAVE_IMAGES, SAVE_DESIRED_IMAGES, SHOW_IMAGES;

// For reading usage from yalm
XmlRpc::XmlRpcValue seg1, seg2, seg3, seg4;
XmlRpc::XmlRpcValue bearing1, bearing2, bearing3, bearing4;
Mat bearing1_points, bearing2_points, bearing3_points, bearing4_points;

// For controlling target detection
bool target1, target2, target3, target4;

// Vector arrays for easy usage
vector<XmlRpc::XmlRpcValue> segmentsXML = {seg1, seg2, seg3, seg4};
vector<XmlRpc::XmlRpcValue> bearingsXML = {bearing1, bearing2, bearing3, bearing4};
vector<Mat> bearings = {bearing1_points, bearing2_points, bearing3_points, bearing4_points};
vector<bool> targets = {target1, target2, target3, target4};

/****************** OPENING STATE PARAMS FROM DESCRIPTORS ******************/
ros::Publisher pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4;
ros::Subscriber pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4;
ros::Subscriber position_sub_1, position_sub_2, position_sub_3, position_sub_4;
ros::Subscriber imu_sub_1, imu_sub_2, imu_sub_3, imu_sub_4;

vector<ros::Publisher> pos_pubs = {pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4};
vector<ros::Subscriber> pos_subs = {pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4};
vector<ros::Subscriber> position_subs = {position_sub_1, position_sub_2, position_sub_3, position_sub_4};
vector<ros::Subscriber> imu_subs = {imu_sub_1, imu_sub_2, imu_sub_3, imu_sub_4};