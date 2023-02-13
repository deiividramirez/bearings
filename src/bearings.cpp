#include "bearings.h"

/****************** DECLARING NAMESPACES ******************/
using namespace cv;
using namespace std;

/****************** MAIN FUNCTION ******************/
int main(int argc, char **argv)
{
	/****************** ROS INITIALIZING ******************/
	ros::init(argc, argv, "bearings");
	ros::NodeHandle nh("ns1"), nh2("ns2"), nh3("ns3"), nh4("ns4"), gen("general");
	vector<ros::NodeHandle> nhs = {nh, nh2, nh3, nh4};
	for (int i = 0; i < 4; i++)
		states[i].load(nhs[i]);

	image_transport::ImageTransport it1(nh);
	image_transport::ImageTransport it2(nh2);
	image_transport::ImageTransport it3(nh3);
	image_transport::ImageTransport it4(nh4);
	image_transport::Subscriber image_sub_1f, image_sub_1b,
		 image_sub_2f, image_sub_2b,
		 image_sub_3f, image_sub_3b,
		 image_sub_4f, image_sub_4b;

	/****************** CREATING PUBLISHER AND SUBSCRIBER ******************/

	gen.getParam("SAVE_DESIRED_IMAGES", SAVE_DESIRED_IMAGES);
	gen.getParam("SAVE_IMAGES", SAVE_IMAGES);
	gen.getParam("SHOW_IMAGES", SHOW_IMAGES);
	gen.getParam("seguimiento_1", seg1);
	gen.getParam("seguimiento_2", seg2);
	gen.getParam("seguimiento_3", seg3);
	gen.getParam("seguimiento_4", seg4);

	gen.getParam("seguimiento_1", segmentsXML[0]);
	gen.getParam("seguimiento_2", segmentsXML[1]);
	gen.getParam("seguimiento_3", segmentsXML[2]);
	gen.getParam("seguimiento_4", segmentsXML[3]);

	gen.getParam("bearing_1", bearingsXML[0]);
	gen.getParam("bearing_2", bearingsXML[1]);
	gen.getParam("bearing_3", bearingsXML[2]);
	gen.getParam("bearing_4", bearingsXML[3]);

	for (int i = 0; i < bearings.size(); i++)
	{
		bearings[i] = convertBearing(bearingsXML[i], segmentsXML[i]);
	}

	cout << "[INFO] SAVE_DESIRED_IMAGES: " << (SAVE_DESIRED_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SAVE_IMAGES: " << (SAVE_IMAGES ? "True" : "False") << endl;
	cout << "[INFO] SHOW_IMAGES: " << (SHOW_IMAGES ? "True\n" : "False\n") << endl;
	cout << "[INFO] seguimiento_1: " << seg1 << endl;
	cout << "[INFO] seguimiento_2: " << seg2 << endl;
	cout << "[INFO] seguimiento_3: " << seg3 << endl;
	cout << "[INFO] seguimiento_4: " << seg4 << endl;

	/****************** FOR SAVING DESIRED IMAGES FROM ACTUAL POSE ******************/
	if (SAVE_DESIRED_IMAGES)
	{
		image_sub_1f = it1.subscribe("/iris_1/camera_front_camera/image_raw", 1, saveDesired1f);
		// image_sub_1b = it1.subscribe("/iris_1/camera_under_camera/image_raw", 1, saveDesired1b);

		image_sub_2f = it2.subscribe("/iris_2/camera_front_camera/image_raw", 1, saveDesired2f);
		// image_sub_2b = it2.subscribe("/iris_2/camera_under_camera/image_raw", 1, saveDesired2b);

		image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, saveDesired3f);
		// image_sub_3b = it3.subscribe("/iris_3/camera_under_camera/image_raw", 1, saveDesired3b);

		image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, saveDesired4f);
		// image_sub_4b = it4.subscribe("/iris_4/camera_under_camera/image_raw", 1, saveDesired4b);
	}
	else
	{
		/****************** FOR CONTROL LAW ******************/
		image_sub_1f = it1.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback);

		image_sub_2f = it2.subscribe("/iris_2/camera_front_camera/image_raw", 1, imageCallback2);

		image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, IMGCallback3);
		// image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, imageCallback3);

		image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, IMGCallback4);
		// image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, imageCallback4);
	}
	ros::Rate rate(30);

	/****************** OPENING DESIRED IMAGES ******************/
	if (!SAVE_DESIRED_IMAGES)
	{
		string image_dir = workspace;
		image_dir += "/src/bearings/src/desired_";

		for (int i = 0; i < 4; i++)
		{
			states[i].desired_configuration.img = imread(image_dir + to_string(i + 1) + "f.jpg", IMREAD_COLOR);
			if (states[i].desired_configuration.img.empty())
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
		for (int i = 0; i < states.size(); i++)
		{
			pos_pubs[i] = nhs[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>("/iris_" + to_string(i + 1) + "/command/trajectory", 1);
			pos_subs[i] = nhs[i].subscribe<geometry_msgs::Pose>("/iris_" + to_string(i + 1) + "/ground_truth/pose", 1, posesCallback[i]);
			imu_subs[i] = nhs[i].subscribe<sensor_msgs::Imu>("/iris_" + to_string(i + 1) + "/imu", 1, imuCallbacks[i]);
			cout << "[INFO] Suscribed and advertised for Iris " << i + 1 << " trajectory and pose." << endl;
		}
	}

	/****************** DATA FOR GRAPHICS ******************/

	vector<float> vel_x1, vel_x2, vel_x3, vel_x4;
	vector<float> vel_y1, vel_y2, vel_y3, vel_y4;
	vector<float> vel_z1, vel_z2, vel_z3, vel_z4;
	vector<float> vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4;
	vector<float> errors1, errors2, errors3, errors4;
	vector<float> errors_pix1, errors_pix2, errors_pix3, errors_pix4;
	vector<float> time1, time2, time3, time4;
	vector<float> lambda1, lambda2, lambda3, lambda4;
	vector<float> x1, x2, x3, x4;
	vector<float> y1, y2, y3, y4;
	vector<float> z1, z2, z3, z4;

	vector<vector<float>> vel_x = {vel_x1, vel_x2, vel_x3, vel_x4};
	vector<vector<float>> vel_y = {vel_y1, vel_y2, vel_y3, vel_y4};
	vector<vector<float>> vel_z = {vel_z1, vel_z2, vel_z3, vel_z4};
	vector<vector<float>> vel_yaw = {vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4};
	vector<vector<float>> errors = {errors1, errors2, errors3, errors4};
	vector<vector<float>> errors_pix = {errors_pix1, errors_pix2, errors_pix3, errors_pix4};
	vector<vector<float>> time = {time1, time2, time3, time4};
	vector<vector<float>> lambda = {lambda1, lambda2, lambda3, lambda4};
	vector<vector<float>> X = {x1, x2, x3, x4};
	vector<vector<float>> Y = {y1, y2, y3, y4};
	vector<vector<float>> Z = {z1, z2, z3, z4};

	/****************** CREATE MESSAGE ******************/
	trajectory_msgs::MultiDOFJointTrajectory msg;
	string file_folder = "/src/bearings/src/data/out/";

	bool first_time = true;

	/****************** STARTING CYCLE ******************/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();

		if (contGEN > 50 && SAVE_DESIRED_IMAGES)
		{
			cout << "\n[INFO] Images have been saved." << endl;
			ros::shutdown();
		}

		if (!states[0].initialized || !states[1].initialized || !states[2].initialized || !states[3].initialized)
		{
			contGEN++;
			rate.sleep();
			cout << "\n[INFO] Waiting for the drones to be initialized..." << endl;
			continue;
		} // if we havent get the new pose for all the drones
		else
		{
			if (first_time)
			{
				first_time = false;
				cout << "\n[INFO] All the drones have been initialized." << endl;
			}

			// FOR TO SAVE DATA IN ARRAY AND CHECK IF THE DRONE IS IN THE TARGET
			for (int i = 0; i < states.size(); i++)
			{
				if (!states[i].in_target)
				{
					time[i].push_back(states[i].t);
					if (i == 2 || i == 3)
					{
						errors[i].push_back((float)states[i].error);
						errors_pix[i].push_back((float)states[i].error);
					}
					else
					{
						errors[i].push_back((float)matching_results[i].mean_feature_error);
						errors_pix[i].push_back((float)matching_results[i].mean_feature_error_pix);
					}
					vel_x[i].push_back(states[i].Vx);
					vel_y[i].push_back(states[i].Vy);
					vel_z[i].push_back(states[i].Vz);
					vel_yaw[i].push_back(states[i].Vyaw);
					lambda[i].push_back(states[i].lambda);

					X[i].push_back(pos_dron[i].pose.position.x);
					Y[i].push_back(pos_dron[i].pose.position.y);
					Z[i].push_back(pos_dron[i].pose.position.z);

					if (matching_results[i].mean_feature_error < states[i].params.feature_threshold)
					{
						cout << "\n[INFO] Target reached within the feature threshold for drone " + to_string(i + 1) << endl;
						states[i].in_target = true;
						break;
					}
				}
				else
				{
					cout << "\n[INFO] Target reached for drone" + to_string(i + 1) << endl;
					break;
				}

				// Publish image of the matching
				// cout << "\n[INFO] Publishing image" << endl;
				// image_pub.publish(image_msg);

				// UPDATE STATE WITH THE CURRENT CONTROL
				auto new_pose = states[i].update();

				// PREPARE MESSAGE
				msg.header.stamp = ros::Time::now();
				mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

				// PUBLISH MESSAGE FOR TRAYECTORY
				pos_pubs[i].publish(msg);
				rate.sleep();
			}
		}

		if (contIMG1 > 1000 || contIMG2 > 1000 || contIMG3 > 1000 || contIMG4 > 1000 || contGEN > 1000)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			break;
		}
		else
		{
			contGEN++;
		}
	}

	// SAVING DATA FOR PYTHON PLOTING
	for (int i = 0; i < states.size(); i++)
	{
		writeFile(errors[i], workspace + file_folder + "out_errors_" + to_string(i + 1) + ".txt");
		writeFile(errors_pix[i], workspace + file_folder + "out_errors_pix_" + to_string(i + 1) + ".txt");
		writeFile(time[i], workspace + file_folder + "out_time_" + to_string(i + 1) + ".txt");
		writeFile(vel_x[i], workspace + file_folder + "out_Vx_" + to_string(i + 1) + ".txt");
		writeFile(vel_y[i], workspace + file_folder + "out_Vy_" + to_string(i + 1) + ".txt");
		writeFile(vel_z[i], workspace + file_folder + "out_Vz_" + to_string(i + 1) + ".txt");
		writeFile(vel_yaw[i], workspace + file_folder + "out_Vyaw_" + to_string(i + 1) + ".txt");
		writeFile(lambda[i], workspace + file_folder + "out_lambda_" + to_string(i + 1) + ".txt");
		writeFile(X[i], workspace + file_folder + "out_X_" + to_string(i + 1) + ".txt");
		writeFile(Y[i], workspace + file_folder + "out_Y_" + to_string(i + 1) + ".txt");
		writeFile(Z[i], workspace + file_folder + "out_Z_" + to_string(i + 1) + ".txt");
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
			  << "--------------------> BEGIN IMGCallback3 for Drone 3<--------------------" << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		Mat actual_bearing, bearing_ground_truth;
		if (getBearing(actual, seg3, actual_bearing, bearing_ground_truth, states[2], 3, pos_dron) < 0)
		{
			cout << "[ERROR] No bearing found" << endl;
			states[2].Vx = 0;
			states[2].Vy = 0;
			states[2].Vz = 0;
			states[2].Vyaw = 0;
		}
		else
		{
			// cout << "Bearing with ground truth drone " << 2 + 1 << endl;
			// cout << bearing_ground_truth << endl;

			// Executing bearing only control
			if (bearingControl(actual_bearing,
									 bearing_ground_truth,
									 bearings[2],
									 states,
									 segmentsXML[2],
									 3,
									 states[2].params.gainv,
									 states[2].params.gainw) < 0)
			{
				cout << "[ERROR] Bearing control failed" << endl;
			}
			else
			{
				cout << "[INFO] Bearing control success" << endl;
				cout << "[INFO] Error: " << states[2].error << endl;
			}
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/3_" + to_string(contIMG3++) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/3_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		cout << "[VELS] Vx: " << states[1].Vx
			  << ", Vy: " << states[1].Vy
			  << ", Vz: " << states[1].Vz
			  << "\nVroll: " << states[1].Vroll
			  << ", Vpitch: " << states[1].Vpitch
			  << ", Wyaw: " << states[1].Vyaw
			  << "\n==> Error: " << matching_results[1].mean_feature_error
			  << "<==" << endl
			  << endl;
		//   << "===================================================================\n\n";
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
			  << "--------------------> BEGIN IMGCallback4 for Drone 4<--------------------" << endl;
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Getting the bearings from camera's drone
		Mat actual_bearing, bearing_ground_truth;
		if (getBearing(actual, seg4, actual_bearing, bearing_ground_truth, states[3], 4, pos_dron) < 0)
		{
			cout << "[ERROR] No bearing found" << endl;
			states[3].Vx = 0;
			states[3].Vy = 0;
			states[3].Vz = 0;
			states[3].Vyaw = 0;
		}
		else
		{
			// cout << "Bearing with ground truth drone " << 3 + 1 << endl;
			// cout << bearing_ground_truth << endl;
			// cout << "bearings[3] = " << bearings[3] << endl;

			// Executing bearing only control
			if (bearingControl(actual_bearing,
									 bearing_ground_truth,
									 bearings[3],
									 states,
									 segmentsXML[3],
									 4,
									 states[3].params.gainv,
									 states[3].params.gainw) < 0)
			{
				cout << "[ERROR] Bearing control failed" << endl;
			}
			else
			{
				cout << "[INFO] Bearing control success" << endl;
				cout << "[INFO] Error: " << states[3].error << endl;
			}
		}

		// Saving images
		string saveIMG;
		if (SAVE_IMAGES)
		{
			saveIMG = "/src/bearings/src/data/img/4_" + to_string(contIMG4++) + ".jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}
		else
		{
			saveIMG = "/src/bearings/src/data/img/4_1.jpg";
			imwrite(workspace + saveIMG, actual);
			// cout << "[INFO] << Image saved >>" << saveIMG << endl;
		}

		cout << "[VELS] Vx: " << states[1].Vx
			  << ", Vy: " << states[1].Vy
			  << ", Vz: " << states[1].Vz
			  << "\nVroll: " << states[1].Vroll
			  << ", Vpitch: " << states[1].Vpitch
			  << ", Wyaw: " << states[1].Vyaw
			  << "\n==> Error: " << matching_results[1].mean_feature_error
			  << "<==" << endl
			  << endl;
		//   << "===================================================================\n\n";
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
			  << "--------------------> BEGIN IMGCallback for Drone 1<--------------------" << endl;
		// cout << "\n[INFO] ImageCallback for drone " << 0 + 1 << endl;

		try
		{
			Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
			// cout << "[INFO] Image received" << endl;

			if (contIMG1++ < 3)
			{
				// 	cout << "\n[INFO] Detecting keypoints" << endl;

				// 	if (states[0].params.camara != 1)
				// 	{
				// 		cout << "[INFO] Detecting points with ORB" << endl;
				// 		if (Compute_descriptors(actual, img_points1, states[0], matching_results[0]) < 0)
				// 		{
				// 			cout << "[ERROR] No keypoints with matches were found" << endl;
				// 			return;
				// 		}
				// 	}
				// 	else
				// 	{
				// 		cout << "[INFO] Detecting points with ArUco" << endl;
				// 		if (aruco_detector(actual, img_points1, states[0], matching_results[0], seg1) < 0)
				// 		{
				// 			cout << "[ERROR] No ArUco were found." << endl;
				// 			// ros::shutdown();
				// 		}
				// 	}
				// 	// cout << "[INFO] img_points1: " << img_points1 << endl;
				// 	// cout << "[INFO] matching_result.p1: " << endl
				// 	// 	  << matching_results[0].p1 << endl;
				// 	// cout << "[INFO] matching_result.p2: " << endl
				// 	// 	  << matching_results[0].p2 << endl
				// 	// 	  << endl;

				// 	img_old1 = actual;
			}
			else
			{
				// Detecting keypoints from ArUco markers
				if (aruco_detector(actual, img_points1, states[0], matching_results[0], seg1) == 0)
				{
					// Executing control law by GUO
					// cout << "[INFO] Calling control law." << endl;
					if (GUO(actual, states[0], matching_results[0]) < 0)
					{
						cout << "[ERROR] Controller failed" << endl;
						return;
					}
					else
					{
						// cout << "[INFO] Controller part has been executed" << endl;
					}
				}
				else
				{
					cout << "[ERROR] No ArUco were found." << endl;
					// ros::shutdown();
					states[0].Vx = 0;
					states[0].Vy = 0;
					states[0].Vz = 0;
					states[0].Vyaw = 0;
				}

				// cout << "POSICIÓN: " << pos_dron[0] << endl;
				// Mat desired_temp, new_points;
				// if (Kanade_Lucas_Tomasi(img_old1, actual, desired_temp, img_points1, states[0], matching_results[0]) < 0)
				// {
				// 	cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				// 	return;
				// }
				// else
				// {
				// 	cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
				// }

				// if (SHOW_IMAGES)
				// {
				// 	imshow("Frontal camera", actual);
				// 	imshow("Desired", desired_temp);
				// 	waitKey(1);
				// }
			}

			// Saving images
			string saveIMG;
			if (SAVE_IMAGES)
			{
				saveIMG = "/src/bearings/src/data/img/1_" + to_string(contIMG1++) + ".jpg";
				imwrite(workspace + saveIMG, actual);
				// cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}
			else
			{
				saveIMG = "/src/bearings/src/data/img/1_1.jpg";
				imwrite(workspace + saveIMG, actual);
				// cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}

			/************************************************************* Prepare message */
			// image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[0].img_matches).toImageMsg();
			// image_msg->header.frame_id = "matching_image";
			// image_msg->width = matching_results[0].img_matches.cols;
			// image_msg->height = matching_results[0].img_matches.rows;
			// image_msg->is_bigendian = false;
			// image_msg->step = sizeof(unsigned char) * matching_results[0].img_matches.cols * 3;
			// image_msg->header.stamp = ros::Time::now();

			/* cout << "[INFO] Matching published" << endl; */

			cout << "[VELS] Vx: " << states[0].Vx
				  << ", Vy: " << states[0].Vy
				  << ", Vz: " << states[0].Vz
				  << "\nVroll: " << states[0].Vroll
				  << ", Vpitch: " << states[0].Vpitch
				  << ", Wyaw: " << states[0].Vyaw
				  << "\n==> Error: " << matching_results[0].mean_feature_error
				  << "<==" << endl
				  << endl;
			//   << "===================================================================\n\n";
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
						 msg->encoding.c_str());
		}
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
			  << "--------------------> BEGIN imageCallback2 for drone 2 <--------------------" << endl;
		// cout << "\n[INFO] ImageCallback function called for drone " << 1 + 1 << endl;

		try
		{
			Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
			// cout << "[INFO] Image received" << endl;

			if (contIMG2++ < 3)
			{
				// cout << "\n[INFO] Detecting keypoints" << endl;

				// if (states[1].params.camara != 1)
				// {
				// 	cout << "[INFO] Detecting points with ORB" << endl;
				// 	if (Compute_descriptors(actual, img_points2, states[1], matching_results[1]) < 0)
				// 	{
				// 		cout << "[ERROR] No keypoints with matches were found" << endl;
				// 		return;
				// 	}
				// }
				// else
				// {
				// 	cout << "[INFO] Detecting points with ArUco" << endl;
				// 	if (aruco_detector(actual, img_points2, states[1], matching_results[1], seg2) < 0)
				// 	{
				// 		cout << "[ERROR] No ArUco were found." << endl;
				// 		// ros::shutdown();
				// 	}
				// }
				// // cout << "[INFO] img_points2: " << img_points2 << endl;
				// cout << "[INFO] matching_result.p1: " << matching_results[1].p1 << endl;
				// cout << "[INFO] matching_result.p2: " << matching_results[1].p2 << endl;

				// img_old2 = actual;
			}
			else
			{
				// Detecting points with ArUco markers
				// cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points2, states[1], matching_results[1], seg2) == 0)
				{
					// Execute control law by GUO
					// cout << "[INFO] Calling control law." << endl;
					if (GUO(actual, states[1], matching_results[1]) < 0)
					{
						cout << "[ERROR] Controller failed" << endl;
						return;
					}
					else
					{
						// cout << "[INFO] Controller part has been executed" << endl;
					}
				}
				else
				{
					cout << "[ERROR] No ArUco were found." << endl;
					// ros::shutdown();
					states[1].Vx = 0;
					states[1].Vy = 0;
					states[1].Vz = 0;
					states[1].Vroll = 0;
				}

				// Mat desired_temp, new_points;
				// if (Kanade_Lucas_Tomasi(img_old2, actual, desired_temp, img_points2, states[1], matching_results[1]) < 0)
				// {
				// 	cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				// 	return;
				// }
				// else
				// {
				// 	cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
				// }

				// if (!SHOW_IMAGES)
				// {
				// 	namedWindow("Desired_2", WINDOW_NORMAL);
				// 	cv::resizeWindow("Desired_2", 960, 540);
				// 	imshow("Desired_2", desired_temp);

				// 	namedWindow("Frontal camera_2", WINDOW_NORMAL);
				// 	cv::resizeWindow("Frontal camera_2", 960, 540);
				// 	imshow("Frontal camera_2", actual);
				// 	waitKey(0);
				// }
			}

			string saveIMG;
			if (SAVE_IMAGES)
			{
				saveIMG = "/src/bearings/src/data/img/2_" + to_string(contIMG2++) + ".jpg";
				imwrite(workspace + saveIMG, actual);
				// cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}
			else
			{
				saveIMG = "/src/bearings/src/data/img/2_1.jpg";
				imwrite(workspace + saveIMG, actual);
				// cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}

			/************************************************************* Prepare message */
			// image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[1].img_matches).toImageMsg();
			// image_msg->header.frame_id = "matching_image";
			// image_msg->width = matching_results[1].img_matches.cols;
			// image_msg->height = matching_results[1].img_matches.rows;
			// image_msg->is_bigendian = false;
			// image_msg->step = sizeof(unsigned char) * matching_results[1].img_matches.cols * 3;
			// image_msg->header.stamp = ros::Time::now();

			/* cout << "[INFO] Matching published" << endl; */

			cout << "[VELS] Vx: " << states[1].Vx
				  << ", Vy: " << states[1].Vy
				  << ", Vz: " << states[1].Vz
				  << "\nVroll: " << states[1].Vroll
				  << ", Vpitch: " << states[1].Vpitch
				  << ", Wyaw: " << states[1].Vyaw
				  << "\n==> Error: " << matching_results[1].mean_feature_error
				  << "<==" << endl
				  << endl;
			//   << "===================================================================\n\n";
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
						 msg->encoding.c_str());
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