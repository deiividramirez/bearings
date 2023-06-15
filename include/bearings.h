/****************** CUSTOM LIBRARIES ******************/
#include "vc_state/vc_state.h"

#ifndef BEARING_TOOLS_H
#define BEARING_TOOLS_H

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

/****************** FUNCTION NAMES ******************/

/****************** FOR WRITING MATRIXES ******************/
void writeFile(vector<float> &vec, const string &name);
void loadImages();

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
void poseCallback5(const geometry_msgs::Pose::ConstPtr &msg);

/****************** FOR BEARINGS CALLBACKS ******************/
// void IMGCallback1(const sensor_msgs::Image::ConstPtr &msg);
// void IMGCallback2(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback4(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback5(const sensor_msgs::Image::ConstPtr &msg);

/****************** FOR IMU CALLBACKS ******************/
// void imuCallback1(const sensor_msgs::Imu::ConstPtr &msg);
// void imuCallback2(const sensor_msgs::Imu::ConstPtr &msg);
// void imuCallback3(const sensor_msgs::Imu::ConstPtr &msg);
// void imuCallback4(const sensor_msgs::Imu::ConstPtr &msg);
// void imuCallback5(const sensor_msgs::Imu::ConstPtr &msg);

void saveStuff(int drone_id);

/****************** FOR CONVERT FROM YALM TO OPENCV MAT ******************/
Mat convertBearing(XmlRpc::XmlRpcValue bearing, XmlRpc::XmlRpcValue segs);

/****************** SAVE REAL POSITION OF DRONES ******************/
// geometry_msgs::PoseStamped pos_dron1, pos_dron2, pos_dron3, pos_dron4, pos_dron5;

/****************** ARRAYS WITH DATA FOR EASY USAGE ******************/
// vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2, imageCallback3, imageCallback4};
vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2};
vector<void (*)(const geometry_msgs::Pose::ConstPtr &)> posesCallback = {poseCallback1, poseCallback2, poseCallback3, poseCallback4, poseCallback5};
// vector<void (*)(const sensor_msgs::Imu::ConstPtr &)> imuCallbacks = {imuCallback1, imuCallback2, imuCallback3, imuCallback4, imuCallback5};
// vector<geometry_msgs::PoseStamped> pos_dron = {pos_dron1, pos_dron2, pos_dron3, pos_dron4, pos_dron5};

/****************** DECLARING OBJECTS TO RECEIVE IMAGE MESSAGES ******************/
sensor_msgs::ImagePtr image_msg;

/****************** WORKSPACE DEFINITION FROM CMAKE ******************/
string workspace = WORKSPACE;

/****************** VISUAL CONTROL STATES AND RESULT VARIABLES ******************/
vc_state state_1, state_2, state_3, state_4, state_5;
// vc_homograpy_matching_result matching_result_1, matching_result_2, matching_result_3, matching_result_4, matching_result_5;

/****************** ARRAYS WITH DATA FOR EASY USAGE ******************/
vector<vc_state> states = {state_1, state_2, state_3, state_4, state_5};
// vector<vc_homograpy_matching_result> matching_results = {matching_result_1, matching_result_2, matching_result_3, matching_result_4, matching_result_5};

/****************** AUXILIAR GLOBAL VARIABLES ******************/

// For saving images and counting them
Mat img_old1, img_points1, img_old2, img_points2, img_old3, img_points3, img_old4, img_points4, img_old5, img_points5;
int contIMG1 = 0, contIMG2 = 0, contIMG3 = 0, contIMG4 = 0, contIMG5 = 0, contGEN = 0, SAVE_IMAGES, SAVE_DESIRED_IMAGES, SHOW_IMAGES;

// For reading usage from yalm
string DRONE_NAME;
int DRONE_COUNT, MODE, INIT_MODE;

double CHANGE_THRESHOLD;

// For controlling target detection
bool target1, target2, target3, target4, target5;
bool change1 = false, change2 = false;

// Vector arrays for easy usage
vector<bool> targets = {target1, target2, target3, target4, target5};

/****************** OPENING STATE PARAMS FROM DESCRIPTORS ******************/
ros::Publisher pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4, pos_pub_5;
ros::Subscriber pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4, pos_sub_5;
ros::Subscriber position_sub_1, position_sub_2, position_sub_3, position_sub_4, position_sub_5;
// ros::Subscriber imu_sub_1, imu_sub_2, imu_sub_3, imu_sub_4, imu_sub_5;

vector<ros::Publisher> pos_pubs = {pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4, pos_pub_5};
vector<ros::Subscriber> pos_subs = {pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4, pos_sub_5};
vector<ros::Subscriber> position_subs = {position_sub_1, position_sub_2, position_sub_3, position_sub_4, position_sub_5};
// vector<ros::Subscriber> imu_subs = {imu_sub_1, imu_sub_2, imu_sub_3, imu_sub_4, imu_sub_5};

/****************** DATA FOR GRAPHICS ******************/
vector<float> vel_x1, vel_x2, vel_x3, vel_x4, vel_x5;
vector<float> vel_y1, vel_y2, vel_y3, vel_y4, vel_y5;
vector<float> vel_z1, vel_z2, vel_z3, vel_z4, vel_z5;
vector<float> vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4, vel_yaw5;
vector<float> errors1, errors2, errors3, errors4, errors5;
vector<float> errors_pix1, errors_pix2, errors_pix3, errors_pix4, errors_pix5;
vector<float> time1, time2, time3, time4, time5;
vector<float> lambda1_kp, lambda2_kp, lambda3_kp, lambda4_kp, lambda5_kp;
vector<float> lambda1_kv, lambda2_kv, lambda3_kv, lambda4_kv, lambda5_kv;
vector<float> lambda1_kd, lambda2_kd, lambda3_kd, lambda4_kd, lambda5_kd;
vector<float> arr_x1, arr_x2, arr_x3, arr_x4, arr_x5;
vector<float> arr_y1, arr_y2, arr_y3, arr_y4, arr_y5;
vector<float> arr_z1, arr_z2, arr_z3, arr_z4, arr_z5;
vector<float> arr_yaw1, arr_yaw2, arr_yaw3, arr_yaw4, arr_yaw5;
vector<float> integral_x1, integral_x2, integral_x3, integral_x4, integral_x5;
vector<float> integral_y1, integral_y2, integral_y3, integral_y4, integral_y5;
vector<float> integral_z1, integral_z2, integral_z3, integral_z4, integral_z5;

vector<vector<float>> vel_x = {vel_x1, vel_x2, vel_x3, vel_x4, vel_x5};
vector<vector<float>> vel_y = {vel_y1, vel_y2, vel_y3, vel_y4, vel_y5};
vector<vector<float>> vel_z = {vel_z1, vel_z2, vel_z3, vel_z4, vel_z5};
vector<vector<float>> vel_yaw = {vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4, vel_yaw5};
vector<vector<float>> errors = {errors1, errors2, errors3, errors4, errors5};
vector<vector<float>> errors_pix = {errors_pix1, errors_pix2, errors_pix3, errors_pix4, errors_pix5};
vector<vector<float>> times = {time1, time2, time3, time4, time5};
vector<vector<float>> lambda_kp = {lambda1_kp, lambda2_kp, lambda3_kp, lambda4_kp, lambda5_kp};
vector<vector<float>> lambda_kv = {lambda1_kv, lambda2_kv, lambda3_kv, lambda4_kv, lambda5_kv};
vector<vector<float>> lambda_kd = {lambda1_kd, lambda2_kd, lambda3_kd, lambda4_kd, lambda5_kd};
vector<vector<float>> X = {arr_x1, arr_x2, arr_x3, arr_x4, arr_x5};
vector<vector<float>> Y = {arr_y1, arr_y2, arr_y3, arr_y4, arr_y5};
vector<vector<float>> Z = {arr_z1, arr_z2, arr_z3, arr_z4, arr_z5};
vector<vector<float>> YAW = {arr_yaw1, arr_yaw2, arr_yaw3, arr_yaw4, arr_yaw5};
vector<vector<float>> integral_x = {integral_x1, integral_x2, integral_x3, integral_x4, integral_x5};
vector<vector<float>> integral_y = {integral_y1, integral_y2, integral_y3, integral_y4, integral_y5};
vector<vector<float>> integral_z = {integral_z1, integral_z2, integral_z3, integral_z4, integral_z5};

double after_t1 = 0, after_t2 = 0, after_t3 = 0, after_t4 = 0, after_t5 = 0;
int LIM_MAX;

#include "vc_state/GUO.h"
GUO guoLider1;
GUO guoLider2;

#include "vc_state/bearingControl.h"
bearingControl bearDrone3;
bearingControl bearDrone4;
bearingControl bearDrone5;

#include "vc_state/RotationalControl.h"
RotationalControl rotDrone1;
RotationalControl rotDrone2;
// RotationalControl rotDrone3;
// RotationalControl rotDrone4;
// RotationalControl rotDrone5;

#endif