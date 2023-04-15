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

void saveStuff(int drone_id);

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
string DRONE_NAME;

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

/****************** DATA FOR GRAPHICS ******************/
vector<float> vel_x1, vel_x2, vel_x3, vel_x4;
vector<float> vel_y1, vel_y2, vel_y3, vel_y4;
vector<float> vel_z1, vel_z2, vel_z3, vel_z4;
vector<float> vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4;
vector<float> errors1, errors2, errors3, errors4;
vector<float> errors_pix1, errors_pix2, errors_pix3, errors_pix4;
vector<float> time1, time2, time3, time4;
vector<float> lambda1_kp, lambda2_kp, lambda3_kp, lambda4_kp;
vector<float> lambda1_kv, lambda2_kv, lambda3_kv, lambda4_kv;
vector<float> lambda1_kd, lambda2_kd, lambda3_kd, lambda4_kd;
vector<float> arr_x1, arr_x2, arr_x3, arr_x4;
vector<float> arr_y1, arr_y2, arr_y3, arr_y4;
vector<float> arr_z1, arr_z2, arr_z3, arr_z4;
vector<float> integral_x1, integral_x2, integral_x3, integral_x4;
vector<float> integral_y1, integral_y2, integral_y3, integral_y4;
vector<float> integral_z1, integral_z2, integral_z3, integral_z4;

vector<vector<float>> vel_x = {vel_x1, vel_x2, vel_x3, vel_x4};
vector<vector<float>> vel_y = {vel_y1, vel_y2, vel_y3, vel_y4};
vector<vector<float>> vel_z = {vel_z1, vel_z2, vel_z3, vel_z4};
vector<vector<float>> vel_yaw = {vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4};
vector<vector<float>> errors = {errors1, errors2, errors3, errors4};
vector<vector<float>> errors_pix = {errors_pix1, errors_pix2, errors_pix3, errors_pix4};
vector<vector<float>> times = {time1, time2, time3, time4};
vector<vector<float>> lambda_kp = {lambda1_kp, lambda2_kp, lambda3_kp, lambda4_kp};
vector<vector<float>> lambda_kv = {lambda1_kv, lambda2_kv, lambda3_kv, lambda4_kv};
vector<vector<float>> lambda_kd = {lambda1_kd, lambda2_kd, lambda3_kd, lambda4_kd};
vector<vector<float>> X = {arr_x1, arr_x2, arr_x3, arr_x4};
vector<vector<float>> Y = {arr_y1, arr_y2, arr_y3, arr_y4};
vector<vector<float>> Z = {arr_z1, arr_z2, arr_z3, arr_z4};
vector<vector<float>> integral_x = {integral_x1, integral_x2, integral_x3, integral_x4};
vector<vector<float>> integral_y = {integral_y1, integral_y2, integral_y3, integral_y4};
vector<vector<float>> integral_z = {integral_z1, integral_z2, integral_z3, integral_z4};

double after_t1 = 0, after_t2 = 0, after_t3 = 0, after_t4 = 0;