#include "bearings.h"

/****************** DECLARING NAMESPACES ******************/
using namespace cv;
using namespace std;

/****************** MAIN FUNCTION ******************/
int main(int argc, char **argv)
{
	cout << RESET_C << endl;
	/****************** ROS INITIALIZING ******************/
	ros::init(argc, argv, "bearings");
	ros::NodeHandle nh("ns1"), nh2("ns2"), nh3("ns3"), nh4("ns4"), nh5("ns5"), gen("general");
	vector<ros::NodeHandle> nhs = {nh, nh2, nh3, nh4, nh5};

	ros::Rate rate(30);

	gen.getParam("MODE", MODE);
	gen.getParam("LIM_MAX", LIM_MAX);
	gen.getParam("DRONE", DRONE_NAME);
	gen.getParam("INIT_MODE", INIT_MODE);
	gen.getParam("DRONE_COUNT", DRONE_COUNT);
	gen.getParam("SAVE_IMAGES", SAVE_IMAGES);
	gen.getParam("SHOW_IMAGES", SHOW_IMAGES);
	gen.getParam("SAVE_DESIRED_IMAGES", SAVE_DESIRED_IMAGES);
	gen.getParam("CHANGE_THRESHOLD_LEADER", CHANGE_THRESHOLD_LEADER);
	gen.getParam("CHANGE_THRESHOLD_FOLLOWER", CHANGE_THRESHOLD_FOLLOWER);

	for (int i = 0; i < DRONE_COUNT; i++)
	{
		states[i].load(nhs[i]);
	}

	cout << GREEN_C << "\n[INFO] All Nodes have been initialized." << RESET_C << endl;

	image_transport::ImageTransport it1(nh), it2(nh2), it3(nh3), it4(nh4), it5(nh5);
	image_transport::Subscriber image_sub_1f, image_sub_1b, image_sub_2f, image_sub_2b, image_sub_3f, image_sub_3b, image_sub_4f, image_sub_4b, image_sub_5f, image_sub_5b;

	/****************** CREATING PUBLISHER AND SUBSCRIBER ******************/

	cout << GREEN_C << "\n\n[INFO] DRONE: " << DRONE_NAME << endl;
	cout << "[INFO] DRONE_COUNT: " << DRONE_COUNT << endl;
	cout << "[INFO] SAVE_DESIRED_IMAGES: " << (SAVE_DESIRED_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SAVE_IMAGES: " << (SAVE_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SHOW_IMAGES: " << (SHOW_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] Max iterations: " << LIM_MAX << RESET_C << endl;

	/****************** FOR SAVING DESIRED IMAGES FROM ACTUAL POSE ******************/
	if (SAVE_DESIRED_IMAGES)
	{
		image_sub_1f = it1.subscribe("/" + DRONE_NAME + "_1/camera_base/image_raw", 1, saveDesired1f);
		image_sub_2f = it2.subscribe("/" + DRONE_NAME + "_2/camera_base/image_raw", 1, saveDesired2f);
		image_sub_3f = it3.subscribe("/" + DRONE_NAME + "_3/camera_base/image_raw", 1, saveDesired3f);
		image_sub_4f = it4.subscribe("/" + DRONE_NAME + "_4/camera_base/image_raw", 1, saveDesired4f);
		if (DRONE_COUNT == 5)
			image_sub_5f = it5.subscribe("/" + DRONE_NAME + "_5/camera_base/image_raw", 1, saveDesired5f);
	}
	else
	{
		/****************** FOR CONTROL LAW ******************/
		loadImages();

		// First leader -> Translational motion (IBVS GUO) and Rotational motion (IBVS CLASSIC)
		image_sub_1f = it1.subscribe("/" + DRONE_NAME + "_1/camera_base/image_raw", 1, imageCallback1);
		guoLider1 = GUO(&states[0], INIT_MODE);
		rotDrone1 = RotationalControl(&states[0]);

		// Second leader -> Translational motion (IBVS GUO) and Rotational motion (IBVS CLASSIC)
		image_sub_2f = it2.subscribe("/" + DRONE_NAME + "_2/camera_base/image_raw", 1, imageCallback2);
		guoLider2 = GUO(&states[1], INIT_MODE);
		rotDrone2 = RotationalControl(&states[1]);

		// First follower -> Translational motion and Rotational motion (BEARING ONLY)
		image_sub_3f = it3.subscribe("/" + DRONE_NAME + "_3/camera_base/image_raw", 1, IMGCallback3);
		bearDrone3 = bearingControl(&states[2], states);

		// Second follower -> Translational motion and Rotational motion (BEARING ONLY)
		image_sub_4f = it4.subscribe("/" + DRONE_NAME + "_4/camera_base/image_raw", 1, IMGCallback4);
		bearDrone4 = bearingControl(&states[3], states);

		// Third follower -> Translational motion and Rotational motion (BEARING ONLY)
		if (DRONE_COUNT == 5)
		{
			image_sub_5f = it5.subscribe("/" + DRONE_NAME + "_5/camera_base/image_raw", 1, IMGCallback5);
			bearDrone5 = bearingControl(&states[4], states);
		}
	}

	/****************** MOVING TO POSES ******************/
	if (!SAVE_DESIRED_IMAGES)
	{
		for (int i = 0; i < DRONE_COUNT; i++)
		{
			pos_pubs[i] = nhs[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/command/trajectory", 1);
			pos_subs[i] = nhs[i].subscribe<geometry_msgs::Pose>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/ground_truth/pose", 1, posesCallback[i]);
			// imu_subs[i] = nhs[i].subscribe<sensor_msgs::Imu>("/" + DRONE_NAME + "_" + to_string(i + 1) + "/imu", 1, imuCallbacks[i]);
			cout << "[INFO] Suscribed and advertised for /" + DRONE_NAME + "_" << i + 1 << " trajectory and pose." << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/command/trajectory" << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/ground_truth/pose" << endl;
			cout << DRONE_NAME + "_" + to_string(i + 1) + "/imu" << endl;
		}
	}

	/****************** STARTING CYCLE ******************/
	bool first_time = true;
	while (ros::ok())
	{
		if (!states[0].initialized || !states[1].initialized || !states[2].initialized || !states[3].initialized)
		{
			if (DRONE_COUNT == 5 && !states[4].initialized)
			{
				contGEN++;
				rate.sleep();
				if (!SAVE_DESIRED_IMAGES)
				{
					cout << "\n[INFO] Waiting for the drones to be initialized..." << endl;
				}
			}
			if (DRONE_COUNT == 4)
			{

				contGEN++;
				rate.sleep();
				if (!SAVE_DESIRED_IMAGES)
				{
					cout << "\n[INFO] Waiting for the drones to be initialized..." << endl;
				}
			}
		}
		else
		{
			if (first_time)
			{
				first_time = false;
				cout << "\n[INFO] All the drones have been initialized." << endl;
			}

			rate.sleep();
		}

		if (contIMG1, contIMG2, contIMG3, contIMG4 > LIM_MAX)
		{
			if ((DRONE_COUNT == 5 && contIMG5 > LIM_MAX) || contGEN > 5 * LIM_MAX)
			{
				cout << RED_C << "[ERROR] No convergence, quitting" << RESET_C << endl;
				break;
			}
			if (DRONE_COUNT == 4 || contGEN > 4 * LIM_MAX)
			{
				cout << RED_C << "[ERROR] No convergence, quitting" << RESET_C << endl;
				break;
			}
		}
		else
		{
			contGEN++;
		}

		// GET A MSG
		ros::spinOnce();

		// SAVING IMAGES
		if (contGEN > 50 && SAVE_DESIRED_IMAGES)
		{
			cout << "\n[INFO] Images have been saved." << endl;
			break;
		}

		if (states[0].in_target && states[1].in_target && states[2].in_target && states[3].in_target)
		{
			if (DRONE_COUNT == 5 && states[4].in_target)
			{
				cout << MAGENTA_C << "\n[INFO] << In target >>" << RESET_C << endl;
				break;
			}
			if (DRONE_COUNT == 4)
			{
				cout << MAGENTA_C << "\n[INFO] << In target >>" << RESET_C << endl;
				break;
			}
		}
	}

	// SAVING DATA FOR PYTHON PLOTING
	if (!SAVE_DESIRED_IMAGES)
	{
		string file_folder = "/src/bearings/src/data/out/";
		for (int i = 0; i < DRONE_COUNT; i++)
		{
			writeFile(times[i], workspace + file_folder + "out_time_" + to_string(i + 1) + ".txt");

			writeFile(errors[i], workspace + file_folder + "out_errors_" + to_string(i + 1) + ".txt");
			writeFile(errors_pix[i], workspace + file_folder + "out_errors_pix_" + to_string(i + 1) + ".txt");

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
			writeFile(YAW[i], workspace + file_folder + "out_YAW_" + to_string(i + 1) + ".txt");

			writeFile(integral_x[i], workspace + file_folder + "out_integral_x_" + to_string(i + 1) + ".txt");
			writeFile(integral_y[i], workspace + file_folder + "out_integral_y_" + to_string(i + 1) + ".txt");
			writeFile(integral_z[i], workspace + file_folder + "out_integral_z_" + to_string(i + 1) + ".txt");

			cout << "\n[SAVED] Data saved for drone " << i + 1 << endl;
		}
	}

	return 0;
}

/*************** CALLBACKS ***************/
/*************** FOR IMAGES ***************/
void imageCallback1(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a visual servoing control with camera's information.
	The control will be executed in the drone's body frame.
	*/
	if (states[0].initialized && !states[0].in_target)
	{
		cout << CYAN_C << endl
			  << "=============> BEGIN IMGCallback for Drone 1 iter: " << contIMG1 << " <=============" << RESET_C << endl;

		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
		if (guoLider1.getVels(actual) == 0)
		{
			cout << GREEN_C "\n[INFO] Controller part has been executed" << RESET_C;
			rotDrone1.getVels(actual);
		}
		else
		{
			cout << RED_C << "[ERROR] No ArUco were found." << RESET_C << endl;
		}

		// cout << ">>>>> Error: " << states[0].error << " >>>>> CHANGE_THRESHOLD_LEADER: " << CHANGE_THRESHOLD_LEADER << " >>>>> MODE: " << MODE << endl;

		if (states[0].error_pix < CHANGE_THRESHOLD_LEADER && states[1].error_pix < CHANGE_THRESHOLD_LEADER && change)
		{
			cout << MAGENTA_C << "[INFO] Leaders drones are in target >>" << RESET_C << endl;
			states[0].in_target = true;
			states[1].in_target = true;
		}

		if (states[0].error_pix < CHANGE_THRESHOLD_LEADER && states[1].error_pix < CHANGE_THRESHOLD_LEADER && !change)
		{
			cout << MAGENTA_C << "\n[INFO] << Leaders drones are in target >>" << RESET_C << endl;
			MODE = 1;
			change = true;
			loadImages();
			CHANGE_THRESHOLD_LEADER -= 0.02;

			guoLider1.changeMode(MODE);
			guoLider2.changeMode(MODE);
		}

		if (SHOW_IMAGES)
		{
			Mat copy = actual.clone();
			for (int i = 0; i < states[0].actual.points.rows; i++)
			{
				circle(copy, Point2f(states[0].actual.points.at<double>(i, 0), states[0].actual.points.at<double>(i, 1)), 10, Scalar(0, 0, 255), -1);
				circle(copy, Point2f(states[0].desired.points.at<double>(i, 0), states[0].desired.points.at<double>(i, 1)), 10, Scalar(0, 255, 0), -1);
			}
			namedWindow("Frontal camera_1", WINDOW_NORMAL);
			cv::resizeWindow("Frontal camera_1", 550, 310);
			imshow("Frontal camera_1", copy);
			waitKey(1);
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/1_" + to_string(contIMG1) + ".jpg";
			imwrite(workspace + saveIMG, actual);
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/1_1.jpg";
			imwrite(workspace + saveIMG, actual);
		}

		contIMG1++;
		// if (contIMG1 % 100 == 0)
		// {
		// 	states[0].integral_error = Mat::zeros(3, 1, CV_64F);
		// 	states[0].integral_error6 = Mat::zeros(6, 1, CV_64F);
		// 	states[0].integral_error12 = Mat::zeros(12, 1, CV_64F);
		// }
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
		cout << CYAN_C << endl
			  << "=============> BEGIN IMGCallback for Drone 2 iter: " << contIMG2 << " <=============" << RESET_C << endl;

		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
		if (guoLider2.getVels(actual) == 0)
		{
			cout << GREEN_C << "\n[INFO] Controller part has been executed" << RESET_C;
			rotDrone2.getVels(actual);
		}
		else
		{
			cout << RED_C << "[ERROR] No ArUco were found." << RESET_C << endl;
		}

		if (states[0].error_pix < CHANGE_THRESHOLD_LEADER && states[1].error_pix < CHANGE_THRESHOLD_LEADER && change)
		{
			cout << MAGENTA_C << "[INFO] Leaders drones are in target >>" << RESET_C << endl;
			states[0].in_target = true;
			states[1].in_target = true;
		}

		if (states[0].error_pix < CHANGE_THRESHOLD_LEADER && states[1].error_pix < CHANGE_THRESHOLD_LEADER && !change)
		{
			cout << MAGENTA_C << "[INFO] Leaders drones are in target >>" << RESET_C << endl;
			MODE = 1;
			change = true;
			loadImages();
			CHANGE_THRESHOLD_LEADER -= 0.02;

			guoLider1.changeMode(MODE);
			guoLider2.changeMode(MODE);
		}

		if (SHOW_IMAGES)
		{
			Mat copy = actual.clone();
			for (int i = 0; i < states[1].actual.points.rows; i++)
			{
				circle(copy, Point2f(states[1].actual.points.at<double>(i, 0), states[1].actual.points.at<double>(i, 1)), 10, Scalar(0, 0, 255), -1);
				circle(copy, Point2f(states[1].desired.points.at<double>(i, 0), states[1].desired.points.at<double>(i, 1)), 10, Scalar(0, 255, 0), -1);
			}
			namedWindow("Frontal camera_2", WINDOW_NORMAL);
			cv::resizeWindow("Frontal camera_2", 550, 310);
			imshow("Frontal camera_2", copy);
			waitKey(1);
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/2_" + to_string(contIMG2) + ".jpg";
			imwrite(workspace + saveIMG, actual);
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/2_1.jpg";
			imwrite(workspace + saveIMG, actual);
		}

		contIMG2++;
		// if (contIMG2 % 100 == 0)
		// {
		// 	states[1].integral_error = Mat::zeros(3, 1, CV_64F);
		// 	states[1].integral_error6 = Mat::zeros(6, 1, CV_64F);
		// 	states[1].integral_error12 = Mat::zeros(12, 1, CV_64F);
		// }
		saveStuff(1);
	}
}

void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg)
{
	/*
	This function is called for suscribed image topic for drone's camera

	This function will executed a bearing only control with camera's obtained
	bearings keeping in mind the error given by the roll, pitch and yaw of the
	drone. The control will be executed in the drone's body frame.
	*/
	if (states[2].initialized && !states[2].in_target)
	{
		cout << CYAN_C << endl
			  << "=============> BEGIN IMGCallback3 for Drone 3 iter: " << contIMG3 << " <=============" << RESET_C << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone3.getVels(actual) < 0)
		{
			cout << RED_C << "[ERROR] Bearing control failed" << RESET_C << endl;
		}
		else
		{
			cout << GREEN_C << "[INFO] Bearing control success" << RESET_C << endl;
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
		if (contIMG3 % 250 == 0)
		{
			states[2].integral_error = Mat::zeros(3, 1, CV_64F);
			states[2].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[2].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(2);

		if (states[2].error < CHANGE_THRESHOLD_FOLLOWER && change)
		{
			cout << MAGENTA_C << "[INFO] Follower drone 3 is in target" << RESET_C << endl;
			states[2].in_target = true;
		}
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
	if (states[3].initialized && !states[3].in_target)
	{
		cout << CYAN_C << endl
			  << "=============> BEGIN IMGCallback4 for Drone 4 iter: " << contIMG4 << " <=============" << RESET_C << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone4.getVels(actual) < 0)
		{
			cout << RED_C << "[ERROR] Bearing control failed" << RESET_C << endl;
		}
		else
		{
			cout << GREEN_C << "[INFO] Bearing control success" << RESET_C << endl;
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
		if (contIMG4 % 250 == 0)
		{
			states[3].integral_error = Mat::zeros(3, 1, CV_64F);
			states[3].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[3].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(3);

		if (states[3].error < CHANGE_THRESHOLD_FOLLOWER && change)
		{
			cout << MAGENTA_C << "[INFO] Follower drone 4 is in target" << RESET_C << endl;
			states[3].in_target = true;
		}
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
	if (states[4].initialized && !states[4].in_target)
	{
		cout << CYAN_C << endl
			  << "=============> BEGIN IMGCallback5 for Drone 5 iter: " << contIMG5 << " <=============" << RESET_C << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		if (bearDrone5.getVels(actual) < 0)
		{
			cout << RED_C << "[ERROR] Bearing control failed" << RESET_C << endl;
		}
		else
		{
			cout << GREEN_C << "[INFO] Bearing control success" << RESET_C << endl;
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
		if (contIMG5 % 250 == 0)
		{
			states[4].integral_error = Mat::zeros(3, 1, CV_64F);
			states[4].integral_error6 = Mat::zeros(6, 1, CV_64F);
			states[4].integral_error12 = Mat::zeros(12, 1, CV_64F);
		}
		saveStuff(4);

		if (states[4].error < CHANGE_THRESHOLD_FOLLOWER && change)
		{
			cout << MAGENTA_C << "[INFO] Follower drone 5 is in target" << RESET_C << endl;
			states[4].in_target = true;
		}
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

	// setting the position if its the first time
	if (!states[0].initialized)
	{
		states[0].X = (float)msg->position.x;
		states[0].Y = (float)msg->position.y;
		states[0].Z = (float)msg->position.z;

		states[0].Roll = (float)roll;
		states[0].Pitch = (float)pitch;
		states[0].Yaw = (float)yaw;

		cout << "\n[INFO] Setting initial position for drone 1" << endl;
		states[0].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);

		states[0].Q = composeR(states[0].Roll, states[0].Pitch, states[0].Yaw);
	}

	states[0].groundTruth.at<double>(0, 0) = (float)msg->position.x;
	states[0].groundTruth.at<double>(1, 0) = (float)msg->position.y;
	states[0].groundTruth.at<double>(2, 0) = (float)msg->position.z;
	states[0].groundTruth.at<double>(3, 0) = (float)roll;
	states[0].groundTruth.at<double>(4, 0) = (float)pitch;
	states[0].groundTruth.at<double>(5, 0) = (float)yaw;
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

	// setting the position if its the first time
	if (!states[1].initialized)
	{
		states[1].X = (float)msg->position.x;
		states[1].Y = (float)msg->position.y;
		states[1].Z = (float)msg->position.z;

		states[1].Roll = (float)roll;
		states[1].Pitch = (float)pitch;
		states[1].Yaw = (float)yaw;

		cout << "\n[INFO] Setting initial position for drone 2" << endl;
		states[1].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);

		states[1].Q = composeR(states[1].Roll, states[1].Pitch, states[1].Yaw);
	}

	states[1].groundTruth.at<double>(0, 0) = (float)msg->position.x;
	states[1].groundTruth.at<double>(1, 0) = (float)msg->position.y;
	states[1].groundTruth.at<double>(2, 0) = (float)msg->position.z;
	states[1].groundTruth.at<double>(3, 0) = (float)roll;
	states[1].groundTruth.at<double>(4, 0) = (float)pitch;
	states[1].groundTruth.at<double>(5, 0) = (float)yaw;
}
void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg)
{
	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained

	// setting the position if its the first time
	if (!states[2].initialized)
	{
		states[2].X = (float)msg->position.x;
		states[2].Y = (float)msg->position.y;
		states[2].Z = (float)msg->position.z;

		states[2].Roll = (float)roll;
		states[2].Pitch = (float)pitch;
		states[2].Yaw = (float)yaw;

		cout << "\n[INFO] Setting initial position for drone 3" << endl;
		states[2].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);

		states[2].Q = composeR(0, 0, states[2].Yaw);

		for (int i = 0; i < states[2].params.seguimiento.rows; i++)
		{
			states[2].Qi.push_back(Mat::zeros(3, 3, CV_64F));
			states[2].Hi.push_back(Mat::zeros(3, 3, CV_64F));
		}
	}

	states[2].groundTruth.at<double>(0, 0) = (float)msg->position.x;
	states[2].groundTruth.at<double>(1, 0) = (float)msg->position.y;
	states[2].groundTruth.at<double>(2, 0) = (float)msg->position.z;
	states[2].groundTruth.at<double>(3, 0) = (float)roll;
	states[2].groundTruth.at<double>(4, 0) = (float)pitch;
	states[2].groundTruth.at<double>(5, 0) = (float)yaw;
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

	// setting the position if its the first time
	if (!states[3].initialized)
	{
		states[3].X = (float)msg->position.x;
		states[3].Y = (float)msg->position.y;
		states[3].Z = (float)msg->position.z;

		states[3].Roll = (float)roll;
		states[3].Pitch = (float)pitch;
		states[3].Yaw = (float)yaw;

		cout << "\n[INFO] Setting initial position for drone 4" << endl;
		states[3].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);

		states[3].Q = composeR(states[3].Roll, states[3].Pitch, states[3].Yaw);

		for (int i = 0; i < states[3].params.seguimiento.rows; i++)
		{
			states[3].Qi.push_back(Mat::zeros(3, 3, CV_64F));
			states[3].Hi.push_back(Mat::zeros(3, 3, CV_64F));
		}
	}

	states[3].groundTruth.at<double>(0, 0) = (float)msg->position.x;
	states[3].groundTruth.at<double>(1, 0) = (float)msg->position.y;
	states[3].groundTruth.at<double>(2, 0) = (float)msg->position.z;
	states[3].groundTruth.at<double>(3, 0) = (float)roll;
	states[3].groundTruth.at<double>(4, 0) = (float)pitch;
	states[3].groundTruth.at<double>(5, 0) = (float)yaw;
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

	// setting the position if its the first time
	if (!states[4].initialized)
	{
		states[4].X = (float)msg->position.x;
		states[4].Y = (float)msg->position.y;
		states[4].Z = (float)msg->position.z;

		states[4].Roll = (float)roll;
		states[4].Pitch = (float)pitch;
		states[4].Yaw = (float)yaw;

		cout << "\n[INFO] Setting initial position for drone 5" << endl;
		states[4].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);

		states[4].Q = composeR(states[4].Roll, states[4].Pitch, states[4].Yaw);

		for (int i = 0; i < states[4].params.seguimiento.rows; i++)
		{
			states[4].Qi.push_back(Mat::zeros(3, 3, CV_64F));
			states[4].Hi.push_back(Mat::zeros(3, 3, CV_64F));
		}
	}

	states[4].groundTruth.at<double>(0, 0) = (float)msg->position.x;
	states[4].groundTruth.at<double>(1, 0) = (float)msg->position.y;
	states[4].groundTruth.at<double>(2, 0) = (float)msg->position.z;
	states[4].groundTruth.at<double>(3, 0) = (float)roll;
	states[4].groundTruth.at<double>(4, 0) = (float)pitch;
	states[4].groundTruth.at<double>(5, 0) = (float)yaw;
}

// FOR TO SAVE DATA IN ARRAY AND CHECK IF THE DRONE IS IN THE TARGET
void saveStuff(int i)
{
	cout << CYAN_C << "\n[INFO] Saving data for drone " + to_string(i + 1) << RESET_C << endl;

	times[i].push_back(states[i].t);

	errors[i].push_back((float)states[i].error);
	errors_pix[i].push_back((float)states[i].error_pix);

	cout << "[INFO] Control error: " << states[i].error << endl;
	cout << "[INFO] Control error in pixels: " << states[i].error_pix << endl;

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
		cout << GREEN_C << "[INFO] Integral error: " << states[i].integral_error6.t() << RESET_C << endl;
	}
	else
	{
		integral_x[i].push_back(states[i].integral_error.at<double>(0, 0));
		integral_y[i].push_back(states[i].integral_error.at<double>(1, 0));
		integral_z[i].push_back(states[i].integral_error.at<double>(2, 0));
		cout << GREEN_C << "[INFO] Integral error: " << states[i].integral_error.t() << RESET_C << endl;
	}

	X[i].push_back(states[i].X);
	Y[i].push_back(states[i].Y);
	Z[i].push_back(states[i].Z);
	YAW[i].push_back(states[i].Yaw);

	// UPDATE STATE WITH THE CURRENT CONTROL
	auto new_pose = states[i].update();

	// PREPARE MESSAGE
	trajectory_msgs::MultiDOFJointTrajectory msg;
	msg.header.stamp = ros::Time::now();
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

	// PUBLISH MESSAGE FOR TRAYECTORY
	pos_pubs[i].publish(msg);
}

void loadImages()
{
	cout << GREEN_C << "\n[INFO] Loading reference images with mode " << MODE << RESET_C << endl;

	string image_dir = workspace;
	image_dir += "/src/bearings/src/desired";

	for (int i = 0; i < DRONE_COUNT; i++)
	{
		states[i].desired.img = imread(image_dir + to_string(i + 1) + "_" + to_string(MODE) + ".jpg", IMREAD_COLOR);
		if (states[i].desired.img.empty())
		{
			cout << RED_C << "[ERROR] Could not open or find the reference image for drone " << i + 1 << RESET_C << endl;
			cout << RED_C << "[ERROR] No dir >> " << image_dir + to_string(i + 1) + "_" + to_string(MODE) + ".jpg" << RESET_C << endl;
			ros::shutdown();
			exit(0);
		}
	}

	cout << GREEN_C << "[INFO] Reference images loaded" << RESET_C << endl;
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

/*************** CONVERT YALM TO OPENCV MAT ***************/
// Mat convertBearing(XmlRpc::XmlRpcValue bearing, XmlRpc::XmlRpcValue segs)
// {
// 	Mat bearingMat = Mat::zeros(3, 3, CV_32F);
// 	if (bearing.size() > 0 && segs.size() > 0)
// 	{
// 		Mat bearingMat = Mat::zeros(3, segs.size(), CV_64F);
// 		if (bearing.getType() == XmlRpc::XmlRpcValue::TypeArray)
// 		{
// 			// cout << "bearing is array" << endl;
// 			for (int i = 0; i < 3; i++)
// 			{
// 				for (int j = 0; j < segs.size(); j++)
// 				{
// 					std::ostringstream ostr;
// 					ostr << bearing[segs.size() * i + j];
// 					std::istringstream istr(ostr.str());
// 					istr >> bearingMat.at<double>(i, j);
// 				}
// 			}
// 		}
// 		return bearingMat;
// 	}
// 	else
// 	{
// 		return Mat();
// 	}
// }

/*************** FOR IMU ***************/
// void imuCallback1(const sensor_msgs::Imu::ConstPtr &msg)
// {
// 	pos_dron[0].pose.orientation.x = (float)msg->orientation.x;
// 	pos_dron[0].pose.orientation.y = (float)msg->orientation.y;
// 	pos_dron[0].pose.orientation.z = (float)msg->orientation.z;
// 	pos_dron[0].pose.orientation.w = (float)msg->orientation.w;

// 	/* pos_dron[0].header.stamp.sec++; */
// }
// void imuCallback2(const sensor_msgs::Imu::ConstPtr &msg)
// {
// 	pos_dron[1].pose.orientation.x = (float)msg->orientation.x;
// 	pos_dron[1].pose.orientation.y = (float)msg->orientation.y;
// 	pos_dron[1].pose.orientation.z = (float)msg->orientation.z;
// 	pos_dron[1].pose.orientation.w = (float)msg->orientation.w;

// 	/* pos_dron[1].header.stamp.sec++; */
// }
// void imuCallback3(const sensor_msgs::Imu::ConstPtr &msg)
// {
// 	/* cout << "IMU 3" << endl; */
// 	pos_dron[2].pose.orientation.x = (float)msg->orientation.x;
// 	pos_dron[2].pose.orientation.y = (float)msg->orientation.y;
// 	pos_dron[2].pose.orientation.z = (float)msg->orientation.z;
// 	pos_dron[2].pose.orientation.w = (float)msg->orientation.w;

// 	/* pos_dron[2].header.stamp.sec++; */
// }
// void imuCallback4(const sensor_msgs::Imu::ConstPtr &msg)
// {
// 	/* cout << "IMU 4" << endl; */
// 	pos_dron[3].pose.orientation.x = (float)msg->orientation.x;
// 	pos_dron[3].pose.orientation.y = (float)msg->orientation.y;
// 	pos_dron[3].pose.orientation.z = (float)msg->orientation.z;
// 	pos_dron[3].pose.orientation.w = (float)msg->orientation.w;

// 	/* pos_dron[3].header.stamp.sec++; */
// }
// void imuCallback5(const sensor_msgs::Imu::ConstPtr &msg)
// {
// 	/* cout << "IMU 4" << endl; */
// 	pos_dron[4].pose.orientation.x = (float)msg->orientation.x;
// 	pos_dron[4].pose.orientation.y = (float)msg->orientation.y;
// 	pos_dron[4].pose.orientation.z = (float)msg->orientation.z;
// 	pos_dron[4].pose.orientation.w = (float)msg->orientation.w;

// 	/* pos_dron[4].header.stamp.sec++; */
// }

int c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0, c7 = 0, c8 = 0, c9 = 0, c10 = 0;

void saveDesired1f(const sensor_msgs::Image::ConstPtr &msg)
{
	// cout << "[INFO] Saving Desired 1f" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

	string tempString = workspace;
	tempString += "/src/bearings/src/desired1_" + to_string(MODE) + ".jpg";
	imwrite(tempString, actual);
	if (c1++ == 1)
		cout << "[INFO] << Image 1 saved >> " << tempString << endl;
}

void saveDesired2f(const sensor_msgs::Image::ConstPtr &msg)
{
	// cout << "[INFO] Saving Desired 2f" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

	string tempString = workspace;
	tempString += "/src/bearings/src/desired2_" + to_string(MODE) + ".jpg";
	imwrite(tempString, actual);
	if (c3++ == 1)
		cout << "[INFO] << Image 2 saved >> " << tempString << endl;
}

void saveDesired3f(const sensor_msgs::Image::ConstPtr &msg)
{
	// cout << "[INFO] Saving Desired 3f" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

	string tempString = workspace;
	tempString += "/src/bearings/src/desired3_" + to_string(MODE) + ".jpg";
	imwrite(tempString, actual);
	if (c5++ == 1)
		cout << "[INFO] << Image 3 saved >> " << tempString << endl;
}

void saveDesired4f(const sensor_msgs::Image::ConstPtr &msg)
{
	// cout << "[INFO] Saving Desired 4f" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

	string tempString = workspace;
	tempString += "/src/bearings/src/desired4_" + to_string(MODE) + ".jpg";
	imwrite(tempString, actual);
	if (c7++ == 1)
		cout << "[INFO] << Image 4 saved >> " << tempString << endl;
}

void saveDesired5f(const sensor_msgs::Image::ConstPtr &msg)
{
	// cout << "[INFO] Saving Desired 4f" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

	string tempString = workspace;
	tempString += "/src/bearings/src/desired5_" + to_string(MODE) + ".jpg";
	imwrite(tempString, actual);
	if (c9++ == 1)
		cout << "[INFO] << Image 5 saved >> " << tempString << endl;
}
