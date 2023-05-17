#include "bearings.h"

/****************** DECLARING NAMESPACES ******************/
using namespace cv;
using namespace std;

/****************** MAIN FUNCTION ******************/
int main(int argc, char **argv)
{
	/****************** ROS INITIALIZING ******************/
	ros::init(argc, argv, "bearings");
	ros::NodeHandle nh("ns1"), nh2("ns2"), nh3("ns3"), nh4("ns4"), nh5("ns5"), gen("general");
	vector<ros::NodeHandle> nhs = {nh, nh2, nh3, nh4, nh5};

	gen.getParam("DRONE", DRONE_NAME);
	gen.getParam("DRONE_COUNT", DRONE_COUNT);

	for (int i = 0; i < DRONE_COUNT; i++)
	{
		states[i].load(nhs[i]);
	}

	cout << "BEARINGS: Node initialized." << endl;

	image_transport::ImageTransport it1(nh);
	image_transport::ImageTransport it2(nh2);
	image_transport::ImageTransport it3(nh3);
	image_transport::ImageTransport it4(nh4);
	image_transport::ImageTransport it5(nh5);
	image_transport::Subscriber image_sub_1f, image_sub_1b,
		 image_sub_2f, image_sub_2b,
		 image_sub_3f, image_sub_3b,
		 image_sub_4f, image_sub_4b,
		 image_sub_5f, image_sub_5b;

	/****************** CREATING PUBLISHER AND SUBSCRIBER ******************/

	gen.getParam("SAVE_DESIRED_IMAGES", SAVE_DESIRED_IMAGES);
	gen.getParam("SAVE_IMAGES", SAVE_IMAGES);
	gen.getParam("SHOW_IMAGES", SHOW_IMAGES);
	gen.getParam("LIM_MAX", LIM_MAX);

	cout << "\n\n[INFO] DRONE: " << DRONE_NAME << endl;
	cout << "[INFO] DRONE_COUNT: " << DRONE_COUNT << endl;
	cout << "[INFO] SAVE_DESIRED_IMAGES: " << (SAVE_DESIRED_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SAVE_IMAGES: " << (SAVE_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SHOW_IMAGES: " << (SHOW_IMAGES ? "True\n" : "False\n") << endl;
	cout << "[INFO] Max iterations: " << LIM_MAX << endl;

	/****************** FOR SAVING DESIRED IMAGES FROM ACTUAL POSE ******************/
	if (SAVE_DESIRED_IMAGES)
	{
		image_sub_1f = it1.subscribe("/" + DRONE_NAME + "_1/camera_base/image_raw", 1, saveDesired1f);
		// image_sub_1b = it1.subscribe("/" +DRONE_NAME + "_1/camera_under_camera/image_raw", 1, saveDesired1b);

		image_sub_2f = it2.subscribe("/" + DRONE_NAME + "_2/camera_base/image_raw", 1, saveDesired2f);
		// image_sub_2b = it2.subscribe("/" +DRONE_NAME + "_2/camera_under_camera/image_raw", 1, saveDesired2b);

		image_sub_3f = it3.subscribe("/" + DRONE_NAME + "_3/camera_base/image_raw", 1, saveDesired3f);
		// image_sub_3b = it3.subscribe("/" +DRONE_NAME + "_3/camera_under_camera/image_raw", 1, saveDesired3b);

		image_sub_4f = it4.subscribe("/" + DRONE_NAME + "_4/camera_base/image_raw", 1, saveDesired4f);
		// image_sub_4b = it4.subscribe("/" +DRONE_NAME + "_4/camera_under_camera/image_raw", 1, saveDesired4b);

		if (DRONE_COUNT == 5)
		{
			image_sub_5f = it5.subscribe("/" + DRONE_NAME + "_5/camera_base/image_raw", 1, saveDesired5f);
			// image_sub_5b = it5.subscribe("/" +DRONE_NAME + "_5/camera_under_camera/image_raw", 1, saveDesired5b);
		}
	}
	else
	{
		/****************** FOR CONTROL LAW ******************/
		image_sub_1f = it1.subscribe("/" + DRONE_NAME + "_1/camera_base/image_raw", 1, imageCallback);
		image_sub_2f = it2.subscribe("/" +DRONE_NAME + "_2/camera_base/image_raw", 1, imageCallback2);

		image_sub_3f = it3.subscribe("/" + DRONE_NAME + "_3/camera_base/image_raw", 1, IMGCallback3);
		image_sub_4f = it4.subscribe("/" + DRONE_NAME + "_4/camera_base/image_raw", 1, IMGCallback4);
		if (DRONE_COUNT == 5)
			image_sub_5f = it5.subscribe("/" + DRONE_NAME + "_5/camera_base/image_raw", 1, IMGCallback5);
	}
	ros::Rate rate(30);

	/****************** OPENING DESIRED IMAGES ******************/
	if (!SAVE_DESIRED_IMAGES)
	{
		string image_dir = workspace;
		image_dir += "/src/bearings/src/desired_";

		for (int i = 0; i < DRONE_COUNT; i++)
		{
			states[i].desired.img = imread(image_dir + to_string(i + 1) + "f.jpg", IMREAD_COLOR);
			if (states[i].desired.img.empty())
			{
				cout << "[ERROR] Could not open or find the reference image for drone " << i + 1 << endl;
				cout << "[ERROR] No dir >> " << image_dir + to_string(i + 1) + "f.jpg" << endl;
				ros::shutdown();
			}
			else
			{
				cout << "[INFO] Reference image for drone " << i + 1 << " has been loaded" << endl;
			}
		}

		guoLider1 = GUO(&states[0]);
		guoLider2 = GUO(&states[1]);

		bearDrone3 = bearingControl(&states[2]);
		bearDrone4 = bearingControl(&states[3]);
		bearDrone5 = bearingControl(&states[4]);

		rotDrone1 = RotationalControl(&states[0]);
		rotDrone2 = RotationalControl(&states[1]);
		rotDrone3 = RotationalControl(&states[2]);
		rotDrone4 = RotationalControl(&states[3]);
		rotDrone5 = RotationalControl(&states[4]);
	}

	/****************** MOVING TO POSES ******************/
	if (!SAVE_DESIRED_IMAGES)
	{
		// FOR TO COMPUTE POINTS IN IMAGE
		// for (int i = 0; i < states.size(); i++)
		// {

		// 	Ptr<ORB> orb = ORB::create(states[i].params.nfeatures,
		// 										states[i].params.scaleFactor,
		// 										states[i].params.nlevels,
		// 										states[i].params.edgeThreshold,
		// 										states[i].params.firstLevel,
		// 										states[i].params.WTA_K,
		// 										states[i].params.scoreType,
		// 										states[i].params.patchSize,
		// 										states[i].params.fastThreshold);

		// 	orb->detect(states[i].desired_configuration.img,
		// 					states[i].desired_configuration.kp);
		// 	orb->compute(states[i].desired_configuration.img,
		// 					 states[i].desired_configuration.kp,
		// 					 states[i].desired_configuration.descriptors);
		// }

		// FOR TO SUSCRIBE, PUBLISH AND ADVERTISE
		for (int i = 0; i < DRONE_COUNT; i++)
		{
			pos_pubs[i] = nhs[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/command/trajectory", 1);
			pos_subs[i] = nhs[i].subscribe<geometry_msgs::Pose>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/ground_truth/pose", 1, posesCallback[i]);
			imu_subs[i] = nhs[i].subscribe<sensor_msgs::Imu>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/imu", 1, imuCallbacks[i]);
			cout << "[INFO] Suscribed and advertised for /" + DRONE_NAME + "_" << i + 1 << " trajectory and pose." << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/command/trajectory" << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/ground_truth/pose" << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/imu" << endl;
		}
	}

	/****************** CREATE MESSAGE ******************/
	string file_folder = "/src/bearings/src/data/out/";

	bool first_time = true;

	/****************** STARTING CYCLE ******************/
	while (ros::ok())
	{
		if (!states[0].initialized || !states[1].initialized || !states[2].initialized || !states[3].initialized)
		{
			contGEN++;
			rate.sleep();
			if (!SAVE_DESIRED_IMAGES)
			{
				cout << "\n[INFO] Waiting for the drones to be initialized..." << endl;
			}
			// continue;
		} // if we havent get the new pose for all the drones
		else
		{
			if (first_time)
			{
				first_time = false;
				cout << "\n[INFO] All the drones have been initialized." << endl;
			}

			rate.sleep();
		}

		if (contIMG1 > LIM_MAX || contIMG2 > LIM_MAX || contIMG3 > LIM_MAX || contIMG4 > LIM_MAX || contGEN > 4 * LIM_MAX)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			ros::shutdown();
		}
		else
		{
			contGEN++;
		}

		// get a msg
		ros::spinOnce();

		if (contGEN > 50 && SAVE_DESIRED_IMAGES)
		{
			cout << "\n[INFO] Images have been saved." << endl;
			ros::shutdown();
		}
	}

	if (!SAVE_DESIRED_IMAGES)
	{
		// SAVING DATA FOR PYTHON PLOTING
		for (int i = 0; i < DRONE_COUNT; i++)
		{
			writeFile(errors[i], workspace + file_folder + "out_errors_" + to_string(i + 1) + ".txt");
			writeFile(errors_pix[i], workspace + file_folder + "out_errors_pix_" + to_string(i + 1) + ".txt");
			writeFile(times[i], workspace + file_folder + "out_time_" + to_string(i + 1) + ".txt");
			writeFile(vel_x[i], workspace + file_folder + "out_Vx_" + to_string(i + 1) + ".txt");
			writeFile(vel_y[i], workspace + file_folder + "out_Vy_" + to_string(i + 1) + ".txt");
			writeFile(vel_z[i], workspace + file_folder + "out_Vz_" + to_string(i + 1) + ".txt");
			writeFile(vel_yaw[i], workspace + file_folder + "out_Vyaw_" + to_string(i + 1) + ".txt");
			writeFile(lambda_kp[i], workspace + file_folder + "out_lambda_kp_" + to_string(i + 1) + ".txt");
			writeFile(lambda_kv[i], workspace + file_folder + "out_lambda_kv_" + to_string(i + 1) + ".txt");
			writeFile(lambda_kd[i], workspace + file_folder + "out_lambda_kd_" + to_string(i + 1) + ".txt");
			writeFile(X[i], workspace + file_folder + "out_X_" + to_string(i + 1) + ".txt");
			writeFile(Y[i], workspace + file_folder + "out_Y_" + to_string(i + 1) + ".txt");
			writeFile(Z[i], workspace + file_folder + "out_Z_" + to_string(i + 1) + ".txt");
			// writeFile(YAW[i], workspace + file_folder + "out_YAW_" + to_string(i + 1) + ".txt");
			// writeFile(roll[i], workspace + file_folder + "out_roll_" + to_string(i + 1) + ".txt");
			// writeFile(pitch[i], workspace + file_folder + "out_pitch_" + to_string(i + 1) + ".txt");
			writeFile(integral_x[i], workspace + file_folder + "out_integral_x_" + to_string(i + 1) + ".txt");
			writeFile(integral_y[i], workspace + file_folder + "out_integral_y_" + to_string(i + 1) + ".txt");
			writeFile(integral_z[i], workspace + file_folder + "out_integral_z_" + to_string(i + 1) + ".txt");

			cout << "[INFO] Data saved for drone " << i + 1 << endl;
		}
	}

	return 0;
}

/*************** CALLBACKS ***************/

/*************** FOR IMAGES ***************/
void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a bearing only control with camera's obtained
	bearings keeping in mind the error given by the roll, pitch and yaw of the
	drone. The control will be executed in the drone's body frame.
	*/
	if (states[2].initialized)
	{
		cout << endl
			  << "=============> BEGIN IMGCallback3 for Drone 3 iter: " << contIMG3 << " <=============" << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone3.getVels(actual) < 0)
		{
			cout << "[ERROR] Bearing control failed" << endl;
		}
		else
		{
			cout << "[INFO] Bearing control success" << endl;
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/3_" + to_string(contIMG3) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/3_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		contIMG3++;
		if (contIMG3 % 100 == 0)
		{
			cout << "============================================" << endl
				  << "[INFO] Resetting integral error" << endl;
			states[2].integral_error = Mat::zeros(3, 1, CV_64F);
			states[2].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[2].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(2);
	}
}
void IMGCallback4(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a bearing only control with camera's obtained
	bearings keeping in mind the error given by the roll, pitch and yaw of the
	drone. The control will be executed in the drone's body frame.
	*/
	if (states[3].initialized)
	{
		cout << endl
			  << "=============> BEGIN IMGCallback4 for Drone 4 iter: " << contIMG4 << " <=============" << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone4.getVels(actual) < 0)
		{
			cout << "[ERROR] Bearing control failed" << endl;
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/4_" + to_string(contIMG4) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/4_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		contIMG4++;
		if (contIMG4 % 100 == 0)
		{
			cout << "============================================" << endl
				  << "[INFO] Resetting integral error" << endl;
			states[3].integral_error = Mat::zeros(3, 1, CV_64F);
			states[3].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[3].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(3);
	}
}
void IMGCallback5(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a bearing only control with camera's obtained
	bearings keeping in mind the error given by the roll, pitch and yaw of the
	drone. The control will be executed in the drone's body frame.
	*/
	if (states[4].initialized)
	{
		cout << endl
			  << "=============> BEGIN IMGCallback5 for Drone 5 iter: " << contIMG5 << " <=============" << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone5.getVels(actual) < 0)
		{
			cout << "[ERROR] Bearing control failed" << endl;
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/5_" + to_string(contIMG5) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/5_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		contIMG5++;
		if (contIMG5 % 100 == 0)
		{
			cout << "============================================" << endl
				  << "[INFO] Resetting integral error" << endl;
			states[4].integral_error = Mat::zeros(3, 1, CV_64F);
			states[4].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[4].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(4);
	}
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a visual servoing control with camera's information.
	The control will be executed in the drone's body frame.
	*/
	if (states[0].initialized && !states[0].in_target)
	{
		cout << endl
			  << "=============> BEGIN IMGCallback for Drone 1 iter: " << contIMG1 << " <=============" << endl;

		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		if (guoLider1.getVels(actual) == 0)
		{
			cout << "\n[INFO] Controller part has been executed" << endl;
			rotDrone1.getVels(actual);
		}
		else
		{
			cout << "[ERROR] No ArUco were found." << endl;
		}

		if (SHOW_IMAGES)
		{
			for (int i = 0; i < states[0].actual.points.rows; i++)
			{
				circle(actual, Point2f(states[0].actual.points.at<double>(i, 0), states[0].actual.points.at<double>(i, 1)), 10, Scalar(0, 0, 255), -1);
				circle(actual, Point2f(states[0].desired.points.at<double>(i, 0), states[0].desired.points.at<double>(i, 1)), 10, Scalar(0, 255, 0), -1);
			}
			namedWindow("Frontal camera_1", WINDOW_NORMAL);
			cv::resizeWindow("Frontal camera_1", 550, 310);
			imshow("Frontal camera_1", actual);
			waitKey(1);
		}
		// }

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/1_" + to_string(contIMG1) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/1_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		contIMG1++;
		if (contIMG1 % 100 == 0)
		{
			states[0].integral_error = Mat::zeros(3, 1, CV_64F);
			states[0].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[0].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(0);
	}
}
void imageCallback2(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a visual servoing control with camera's information.
	The control will be executed in the drone's body frame.
	*/
	if (states[1].initialized && !states[1].in_target)
	{
		cout << endl
			  << "=============> BEGIN IMGCallback for Drone 2 iter: " << contIMG2 << " <=============" << endl;

		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		if (guoLider2.getVels(actual) == 0)
		{
			cout << "\n[INFO] Controller part has been executed" << endl;
			rotDrone2.getVels(actual);
		}
		else
		{
			cout << "[ERROR] No ArUco were found." << endl;
		}

		if (SHOW_IMAGES)
		{
			for (int i = 0; i < states[1].actual.points.rows; i++)
			{
				circle(actual, Point2f(states[1].actual.points.at<double>(i, 0), states[1].actual.points.at<double>(i, 1)), 10, Scalar(0, 0, 255), -1);
				circle(actual, Point2f(states[1].desired.points.at<double>(i, 0), states[1].desired.points.at<double>(i, 1)), 10, Scalar(0, 255, 0), -1);
			}
			namedWindow("Frontal camera_2", WINDOW_NORMAL);
			cv::resizeWindow("Frontal camera_2", 550, 310);
			imshow("Frontal camera_2", actual);
			waitKey(1);
		}
		// }

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/2_" + to_string(contIMG2) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/2_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		contIMG2++;
		if (contIMG2 % 100 == 0)
		{
			states[1].integral_error = Mat::zeros(3, 1, CV_64F);
			states[1].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[1].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(1);
	}
}

/*************** FOR POSES ***************/
void poseCallback1(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl
	// 	  << "[INFO] poseCallback function for drone 1" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[0].Roll = (float)roll;
	states[0].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[0].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[0].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}

	pos_dron[0].pose.position.x = (float)msg->position.x;
	pos_dron[0].pose.position.y = (float)msg->position.y;
	pos_dron[0].pose.position.z = (float)msg->position.z;

	pos_dron[0].header.seq++;
	pos_dron[0].header.stamp = ros::Time::now();
	pos_dron[0].header.frame_id = "world";
}
void poseCallback2(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl
	// 	  << "[INFO] poseCallback function for drone 2" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[1].Roll = (float)roll;
	states[1].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[1].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[1].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}

	pos_dron[1].pose.position.x = (float)msg->position.x;
	pos_dron[1].pose.position.y = (float)msg->position.y;
	pos_dron[1].pose.position.z = (float)msg->position.z;

	pos_dron[1].header.seq++;
	pos_dron[1].header.stamp = ros::Time::now();
	pos_dron[1].header.frame_id = "world";
}
void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl
	// 	  << "[INFO] poseCallback function for drone 3" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[2].Roll = (float)roll;
	states[2].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[2].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[2].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}

	pos_dron[2].pose.position.x = (float)msg->position.x;
	pos_dron[2].pose.position.y = (float)msg->position.y;
	pos_dron[2].pose.position.z = (float)msg->position.z;

	pos_dron[2].header.seq++;
	pos_dron[2].header.stamp = ros::Time::now();
	pos_dron[2].header.frame_id = "world";
}
void poseCallback4(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl
	// 	  << "[INFO] poseCallback function for drone 4" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[3].Roll = (float)roll;
	states[3].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[3].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[3].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}

	pos_dron[3].pose.position.x = (float)msg->position.x;
	pos_dron[3].pose.position.y = (float)msg->position.y;
	pos_dron[3].pose.position.z = (float)msg->position.z;

	pos_dron[3].header.seq++;
	pos_dron[3].header.stamp = ros::Time::now();
	pos_dron[3].header.frame_id = "world";
}
void poseCallback5(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl
	// 	  << "[INFO] poseCallback function for drone 4" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[4].Roll = (float)roll;
	states[4].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[4].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[4].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}

	pos_dron[4].pose.position.x = (float)msg->position.x;
	pos_dron[4].pose.position.y = (float)msg->position.y;
	pos_dron[4].pose.position.z = (float)msg->position.z;

	pos_dron[4].header.seq++;
	pos_dron[4].header.stamp = ros::Time::now();
	pos_dron[4].header.frame_id = "world";
}

/*************** FOR IMU ***************/
void imuCallback1(const sensor_msgs::Imu::ConstPtr &msg)
{
	pos_dron[0].pose.orientation.x = (float)msg->orientation.x;
	pos_dron[0].pose.orientation.y = (float)msg->orientation.y;
	pos_dron[0].pose.orientation.z = (float)msg->orientation.z;
	pos_dron[0].pose.orientation.w = (float)msg->orientation.w;

	/* pos_dron[0].header.stamp.sec++; */
}
void imuCallback2(const sensor_msgs::Imu::ConstPtr &msg)
{
	pos_dron[1].pose.orientation.x = (float)msg->orientation.x;
	pos_dron[1].pose.orientation.y = (float)msg->orientation.y;
	pos_dron[1].pose.orientation.z = (float)msg->orientation.z;
	pos_dron[1].pose.orientation.w = (float)msg->orientation.w;

	/* pos_dron[1].header.stamp.sec++; */
}
void imuCallback3(const sensor_msgs::Imu::ConstPtr &msg)
{
	/* cout << "IMU 3" << endl; */
	pos_dron[2].pose.orientation.x = (float)msg->orientation.x;
	pos_dron[2].pose.orientation.y = (float)msg->orientation.y;
	pos_dron[2].pose.orientation.z = (float)msg->orientation.z;
	pos_dron[2].pose.orientation.w = (float)msg->orientation.w;

	/* pos_dron[2].header.stamp.sec++; */
}
void imuCallback4(const sensor_msgs::Imu::ConstPtr &msg)
{
	/* cout << "IMU 4" << endl; */
	pos_dron[3].pose.orientation.x = (float)msg->orientation.x;
	pos_dron[3].pose.orientation.y = (float)msg->orientation.y;
	pos_dron[3].pose.orientation.z = (float)msg->orientation.z;
	pos_dron[3].pose.orientation.w = (float)msg->orientation.w;

	/* pos_dron[3].header.stamp.sec++; */
}
void imuCallback5(const sensor_msgs::Imu::ConstPtr &msg)
{
	/* cout << "IMU 4" << endl; */
	pos_dron[4].pose.orientation.x = (float)msg->orientation.x;
	pos_dron[4].pose.orientation.y = (float)msg->orientation.y;
	pos_dron[4].pose.orientation.z = (float)msg->orientation.z;
	pos_dron[4].pose.orientation.w = (float)msg->orientation.w;

	/* pos_dron[4].header.stamp.sec++; */
}

void saveStuff(int i)
{
	// FOR TO SAVE DATA IN ARRAY AND CHECK IF THE DRONE IS IN THE TARGET
	if (!states[i].in_target)
	{
		cout << "\n[INFO] Saving data for drone " + to_string(i + 1) << endl;
		cout << "[VELS] Vx: " << states[i].Vx
			  << ", Vy: " << states[i].Vy
			  << ", Vz: " << states[i].Vz
			  << "\nVroll: " << states[i].Vroll
			  << ", Vpitch: " << states[i].Vpitch
			  << ", Vyaw: " << states[i].Vyaw
			  << "\n==> Error: " << states[i].error
			  << "<==" << endl
			  << endl;

		times[i].push_back(states[i].t);
		if (i == 0 || i == 1)
		{
			// errors[i].push_back((float)matching_results[i].mean_feature_error);
			// errors_pix[i].push_back((float)matching_results[i].mean_feature_error_pix);
			errors[i].push_back((float)states[i].error);
			errors_pix[i].push_back((float)states[i].error_pix);
			// cout << "[INFO] Error: " << matching_results[i].mean_feature_error << endl;
			cout << "[INFO] Error pix: " << states[i].error_pix << endl;
		}
		else
		{
			errors[i].push_back((float)states[i].error);
			errors_pix[i].push_back((float)states[i].error);
			cout << "[INFO] Error: " << states[i].error << endl;
		}

		// states[i].Vyaw = -states[i].Yaw;

		vel_x[i].push_back(states[i].Vx);
		vel_y[i].push_back(states[i].Vy);
		vel_z[i].push_back(states[i].Vz);
		vel_yaw[i].push_back(states[i].Vyaw);
		lambda_kp[i].push_back(states[i].lambda_kp);
		lambda_kv[i].push_back(states[i].lambda_kv);
		lambda_kd[i].push_back(states[i].lambda_kd);

		if (i == 0 || i == 1)
		{
			integral_x[i].push_back((states[i].integral_error6.at<double>(0, 0) + states[i].integral_error6.at<double>(1, 0)) / 2.);
			integral_y[i].push_back((states[i].integral_error6.at<double>(2, 0) + states[i].integral_error6.at<double>(3, 0)) / 2.);
			integral_z[i].push_back((states[i].integral_error6.at<double>(4, 0) + states[i].integral_error6.at<double>(5, 0)) / 2.);
			cout << "[INFO] Integral error: " << states[i].integral_error6.t() << endl;
		}
		else
		{
			integral_x[i].push_back(states[i].integral_error.at<double>(0, 0));
			integral_y[i].push_back(states[i].integral_error.at<double>(1, 0));
			integral_z[i].push_back(states[i].integral_error.at<double>(2, 0));
			cout << "[INFO] Integral error: " << states[i].integral_error.t() << endl;
		}

		X[i].push_back(pos_dron[i].pose.position.x);
		Y[i].push_back(pos_dron[i].pose.position.y);
		Z[i].push_back(pos_dron[i].pose.position.z);

		// if (matching_results[i].mean_feature_error < states[i].params.feature_threshold)
		// {
		// 	cout << "\n[INFO] Target reached within the feature threshold for drone " + to_string(i + 1) << endl;
		// 	states[i].in_target = true;
		// 	// break;
		// }

		// Publish image of the matching
		// cout << "\n[INFO] Publishing image" << endl;
		// image_pub.publish(image_msg);

		// UPDATE STATE WITH THE CURRENT CONTROL
		auto new_pose = states[i].update();

		// PREPARE MESSAGE
		trajectory_msgs::MultiDOFJointTrajectory msg;
		msg.header.stamp = ros::Time::now();
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

		// PUBLISH MESSAGE FOR TRAYECTORY
		pos_pubs[i].publish(msg);
	}
	else
	{
		// cout << "\n[INFO] Target reached for drone" + to_string(i + 1) << endl;
		// break;
	}
}

/*************** CONVERT YALM TO OPENCV MAT ***************/
Mat convertBearing(XmlRpc::XmlRpcValue bearing, XmlRpc::XmlRpcValue segs)
{
	Mat bearingMat = Mat::zeros(3, 3, CV_32F);
	if (bearing.size() > 0 && segs.size() > 0)
	{
		Mat bearingMat = Mat::zeros(3, segs.size(), CV_64F);
		if (bearing.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			// cout << "bearing is array" << endl;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < segs.size(); j++)
				{
					std::ostringstream ostr;
					ostr << bearing[segs.size() * i + j];
					std::istringstream istr(ostr.str());
					istr >> bearingMat.at<double>(i, j);
				}
			}
		}
		return bearingMat;
	}
	else
	{
		return Mat();
	}
}

/*************** SAVE MATRIX TO FILE ***************/
void writeFile(vector<float> &vec, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < vec.size(); i++)
		myfile << vec[i] << endl;
	myfile.close();
}
void writeMatrix(Mat &mat, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			myfile << mat.at<float>(i, j) << " ";
		}
		myfile << endl;
	}
	myfile.close();
}