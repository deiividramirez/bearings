#ifndef VC_CONTROLLER_H
#define VC_CONTROLLER_H

/******************************************************* ROS libraries*/
#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

/**************************************************** OpenCv Libraries*/
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

/*************************************************** c++ libraries */
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <iostream>

/*************************************************** custom libraries */
#include "vc_state/vc_state.h"

#endif
