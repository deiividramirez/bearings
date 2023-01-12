#include "bearings.h"

/****************** DECLARING NAMESPACES ******************/
using namespace cv;
using namespace std;

/****************** CALLBACKS ******************/
void writeFile(vector<float> &vec, const string &name);

void doNothing(const sensor_msgs::Image::ConstPtr &msg);

void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void imageCallback2(const sensor_msgs::Image::ConstPtr &msg);
void imageCallback3(const sensor_msgs::Image::ConstPtr &msg);
void imageCallback4(const sensor_msgs::Image::ConstPtr &msg);

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback1(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback2(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback4(const geometry_msgs::Pose::ConstPtr &msg);

// array with functions
vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2, imageCallback3, imageCallback4};
vector<void (*)(const geometry_msgs::Pose::ConstPtr &)> posesCallback = {poseCallback1, poseCallback2, poseCallback3, poseCallback4};

/****************** DECLARING OBJECTS TO RECEIVE MESSAGES ******************/
sensor_msgs::ImagePtr image_msg;

/****************** WORKSPACE DEFINITION FROM CMAKE ******************/
string workspace = WORKSPACE;

/****************** VISUAL CONTROL STATE AND RESULTS VARIABLES ******************/
vc_state state_1, state_2, state_3, state_4;
vc_homograpy_matching_result matching_result_1, matching_result_2, matching_result_3, matching_result_4;

vector<vc_state> states = {state_1, state_2, state_3, state_4};
vector<vc_homograpy_matching_result> matching_results = {matching_result_1, matching_result_2, matching_result_3, matching_result_4};

/****************** AUXILIAR GLOBAL VARIABLES ******************/
Mat img_old1, img_points1, img_old2, img_points2, img_old3, img_points3, img_old4, img_points4;
int contIMG1 = 0, contIMG2 = 0, contIMG3 = 0, contIMG4 = 0, contGEN = 0;

/****************** MAIN FUNCTION ******************/
int main(int argc, char **argv)
{
	/****************** ROS INITIALIZING ******************/
	ros::init(argc, argv, "bearings");
	ros::NodeHandle nh("ns1"), nh2("ns2"), nh3("ns3"), nh4("ns4"), gen("general");
	vector<ros::NodeHandle> nhs = {nh, nh2, nh3, nh4};
	for (int i = 0; i < 4; i++)
		states[i].load(nhs[i]);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub_1f, image_sub_1b,
		 image_sub_2f, image_sub_2b,
		 image_sub_3f, image_sub_3b,
		 image_sub_4f, image_sub_4b;

	/****************** CREATING PUBLISHER AND SUBSCRIBER ******************/

	int SAVE_DESIRED_IMAGES;
	gen.getParam("SAVE_DESIRED_IMAGES", SAVE_DESIRED_IMAGES);

	cout << "\n[INFO] SAVE_DESIRED_IMAGES: " << (SAVE_DESIRED_IMAGES ? "True\n" : "False\n") << endl;

	/****************** FOR SAVING DESIRED IMAGES FROM ACTUAL POSE ******************/
	if (SAVE_DESIRED_IMAGES)
	{
		image_sub_1f = it.subscribe("/iris_1/camera_front_camera/image_raw", 1, saveDesired1f);
		image_sub_1b = it.subscribe("/iris_1/camera_under_camera/image_raw", 1, saveDesired1b);

		image_sub_2f = it.subscribe("/iris_2/camera_front_camera/image_raw", 1, saveDesired2f);
		image_sub_2b = it.subscribe("/iris_2/camera_under_camera/image_raw", 1, saveDesired2b);

		image_sub_3f = it.subscribe("/iris_3/camera_front_camera/image_raw", 1, saveDesired3f);
		image_sub_3b = it.subscribe("/iris_3/camera_under_camera/image_raw", 1, saveDesired3b);

		image_sub_4f = it.subscribe("/iris_4/camera_front_camera/image_raw", 1, saveDesired4f);
		image_sub_4b = it.subscribe("/iris_4/camera_under_camera/image_raw", 1, saveDesired4b);
	}
	else
	{
		/****************** FOR CONTROL LAW ******************/
		image_sub_1f = it.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback);
		image_sub_1b = it.subscribe("/iris_1/camera_under_camera/image_raw", 1, doNothing);

		image_sub_2f = it.subscribe("/iris_2/camera_front_camera/image_raw", 1, imageCallback2);
		image_sub_2b = it.subscribe("/iris_2/camera_under_camera/image_raw", 1, doNothing);

		image_sub_3f = it.subscribe("/iris_3/camera_front_camera/image_raw", 1, imageCallback3);
		image_sub_3b = it.subscribe("/iris_3/camera_under_camera/image_raw", 1, doNothing);

		image_sub_4f = it.subscribe("/iris_4/camera_front_camera/image_raw", 1, imageCallback4);
		image_sub_4b = it.subscribe("/iris_4/camera_under_camera/image_raw", 1, doNothing);
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

	/****************** OPENING STATE PARAMS FROM DESCRIPTORS ******************/
	/****************** MOVING TO POSES ******************/
	ros::Publisher pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4;
	ros::Subscriber pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4;

	vector<ros::Publisher> pos_pubs = {pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4};
	vector<ros::Subscriber> pos_subs = {pos_sub_1, pos_sub_2, pos_sub_3, pos_sub_4};

	if (!SAVE_DESIRED_IMAGES)
	{
		for (int i = 0; i < states.size(); i++)
		{

			Ptr<ORB> orb = ORB::create(states[i].params.nfeatures,
												states[i].params.scaleFactor,
												states[i].params.nlevels,
												states[i].params.edgeThreshold,
												states[i].params.firstLevel,
												states[i].params.WTA_K,
												states[i].params.scoreType,
												states[i].params.patchSize,
												states[i].params.fastThreshold);

			orb->detect(states[i].desired_configuration.img,
							states[i].desired_configuration.kp);
			orb->compute(states[i].desired_configuration.img,
							 states[i].desired_configuration.kp,
							 states[i].desired_configuration.descriptors);
		}

		for (int i = 0; i < states.size(); i++)
		{
			pos_pubs[i] = nhs[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>("/iris_" + to_string(i + 1) + "/command/trajectory", 1);
			pos_subs[i] = nhs[i].subscribe<geometry_msgs::Pose>("/iris_" + to_string(i + 1) + "/ground_truth/pose", 1, posesCallback[i]);
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

	vector<vector<float>> vel_x = {vel_x1, vel_x2, vel_x3, vel_x4};
	vector<vector<float>> vel_y = {vel_y1, vel_y2, vel_y3, vel_y4};
	vector<vector<float>> vel_z = {vel_z1, vel_z2, vel_z3, vel_z4};
	vector<vector<float>> vel_yaw = {vel_yaw1, vel_yaw2, vel_yaw3, vel_yaw4};
	vector<vector<float>> errors = {errors1, errors2, errors3, errors4};
	vector<vector<float>> errors_pix = {errors_pix1, errors_pix2, errors_pix3, errors_pix4};
	vector<vector<float>> time = {time1, time2, time3, time4};
	vector<vector<float>> lambda = {lambda1, lambda2, lambda3, lambda4};

	/****************** CREATE MESSAGE ******************/
	trajectory_msgs::MultiDOFJointTrajectory msg;
	string file_folder = "/src/bearings/src/data/";

	/****************** STARTING CYCLE ******************/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();

		if (contGEN > 10 && SAVE_DESIRED_IMAGES)
		{
			cout << "[INFO] Images have been saved." << endl;
			ros::shutdown();
		}

		if (!states[0].initialized || !states[1].initialized || !states[2].initialized || !states[3].initialized)
		{
			contGEN++;
			rate.sleep();
			continue;
		} // if we havent get the new pose for all the drones

		/****************** SAVE DATA ******************/
		for (int i = 0; i < states.size(); i++)
		{
			if (!states[i].in_target)
			{
				time[i].push_back(states[i].t);
				errors[i].push_back((float)matching_results[i].mean_feature_error);
				errors_pix[i].push_back((float)matching_results[i].mean_feature_error_pix);
				vel_x[i].push_back(states[i].Vx);
				vel_y[i].push_back(states[i].Vy);
				vel_z[i].push_back(states[i].Vz);
				vel_yaw[i].push_back(states[i].Vyaw);
				lambda[i].push_back(states[i].lambda);

				if (matching_results[i].mean_feature_error < states[i].params.feature_threshold)
				{
					cout << "\n[INFO] Target reached within the feature threshold for drone" + to_string(i + 1) << endl;
					waitKey(0);
					break;
				}
			}
			else
			{
				cout << "\n[INFO] Target reached for drone" + to_string(i + 1) << endl;
				waitKey(0);
				break;
			}

			// Publish image of the matching
			// cout << "\n[INFO] Publishing image" << endl;
			// image_pub.publish(image_msg);

			// Update state with the current control
			auto new_pose = states[i].update();

			// Prepare msg
			msg.header.stamp = ros::Time::now();
			mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

			// Publish msg
			pos_pubs[i].publish(msg);
		}

		rate.sleep();
		if (contIMG1 > 1000 || contIMG2 > 1000 || contIMG3 > 1000 || contIMG4 > 1000)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			break;
		}
	}

	// save data
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
	}

	return 0;
}

/*
	function: imageCallback
	description: uses the msg image and converts it to and opencv image to obtain the kp and
	descriptors, it is done if the drone has moved to the defined position. After that the resulting image and velocities are published.
	params:
		msg: ptr to the msg image.
*/

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback function called for drone " << 0 + 1 << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
		cout << "[INFO] Image received" << endl;

		if (contIMG1 < 3)
		{
			contIMG1++;
			cout << "\n[INFO] Detecting keypoints" << endl;

			if (states[0].params.camara != 1)
			{
				cout << "[INFO] Detecting points with ORB" << endl;
				if (Compute_descriptors(actual, img_points1, states[0], matching_results[0]) < 0)
				{
					cout << "[ERROR] No keypoints with matches were found" << endl;
					return;
				}
			}
			else
			{
				cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points1, states[0], matching_results[0]) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
				}
			}
			// cout << "[INFO] img_points1: " << img_points1 << endl;
			cout << "[INFO] matching_result.p1: " << endl
				  << matching_results[0].p1 << endl;
			cout << "[INFO] matching_result.p2: " << endl
				  << matching_results[0].p2 << endl
				  << endl;

			img_old1 = actual;
			cout << "PASA DEL ELSE ?" << endl;
		}
		else
		{
			img_new = actual;

			cout << "[INFO] Calling control law." << endl;
			if (GUO(actual, states[0], matching_results[0]) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			Mat desired_temp, new_points;
			if (Kanade_Lucas_Tomasi(img_old1, img_new, desired_temp, img_points1, states[0], matching_results[0]) < 0)
			{
				cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
			}

			imshow("Frontal camera", img_new);
			imshow("Desired", desired_temp);
			waitKey(1);
		}

		/* string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG1++) + ".jpg";
		imwrite(workspace + saveIMG, img_new);
		cout << "[INFO] << Image saved >>" << saveIMG << endl; */

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[0].img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_results[0].img_matches.cols;
		image_msg->height = matching_results[0].img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_results[0].img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (states[0].initialized)
			cout << "[VELS] Vx: " << states[0].Vx
				  << ", Vy: " << states[0].Vy
				  << ", Vz: " << states[0].Vz
				  << "\nVroll: " << states[0].Vroll
				  << ", Vpitch: " << states[0].Vpitch
				  << ", Wyaw: " << states[0].Vyaw
				  << "\n==> average error: " << matching_results[0].mean_feature_error
				  << "<==" << endl
				  << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
					 msg->encoding.c_str());
	}
}

void imageCallback2(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback function called for drone " << 1 + 1 << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
		cout << "[INFO] Image received" << endl;

		if (contIMG2 < 3)
		{
			contIMG2++;
			cout << "\n[INFO] Detecting keypoints" << endl;

			if (states[1].params.camara != 1)
			{
				cout << "[INFO] Detecting points with ORB" << endl;
				if (Compute_descriptors(actual, img_points2, states[1], matching_results[1]) < 0)
				{
					cout << "[ERROR] No keypoints with matches were found" << endl;
					return;
				}
			}
			else
			{
				cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points2, states[1], matching_results[1]) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
				}
			}
			// cout << "[INFO] img_points2: " << img_points2 << endl;
			cout << "[INFO] matching_result.p1: " << matching_results[1].p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_results[1].p2 << endl;

			img_old2 = actual;
		}
		else
		{
			cout << "PASA DEL ELSE ?" << endl;
			img_new = actual;

			cout << "[INFO] Calling control law." << endl;
			if (GUO(actual, states[1], matching_results[1]) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			Mat desired_temp, new_points;
			if (Kanade_Lucas_Tomasi(img_old2, img_new, desired_temp, img_points2, states[1], matching_results[1]) < 0)
			{
				cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
			}

			imshow("Frontal camera", img_new);
			imshow("Desired", desired_temp);
			waitKey(1);
		}

		/* string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG2++) + ".jpg";
		imwrite(workspace + saveIMG, img_new);
		cout << "[INFO] << Image saved >>" << saveIMG << endl; */

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[1].img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_results[1].img_matches.cols;
		image_msg->height = matching_results[1].img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_results[1].img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (states[1].initialized)
			cout << "[VELS] Vx: " << states[1].Vx
				  << ", Vy: " << states[1].Vy
				  << ", Vz: " << states[1].Vz
				  << "\nVroll: " << states[1].Vroll
				  << ", Vpitch: " << states[1].Vpitch
				  << ", Wyaw: " << states[1].Vyaw
				  << "\n==> average error: " << matching_results[1].mean_feature_error
				  << "<==" << endl
				  << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
					 msg->encoding.c_str());
	}
}

void imageCallback3(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback function called for drone " << 2 + 1 << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
		cout << "[INFO] Image received" << endl;

		if (contIMG3 < 3)
		{
			contIMG3++;
			cout << "\n[INFO] Detecting keypoints" << endl;

			if (states[2].params.camara != 1)
			{
				cout << "[INFO] Detecting points with ORB" << endl;
				if (Compute_descriptors(actual, img_points3, states[2], matching_results[2]) < 0)
				{
					cout << "[ERROR] No keypoints with matches were found" << endl;
					return;
				}
			}
			else
			{
				cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points3, states[2], matching_results[2]) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
				}
			}
			// cout << "[INFO] img_points3: " << img_points3 << endl;
			cout << "[INFO] matching_result.p1: " << matching_results[2].p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_results[2].p2 << endl;

			img_old3 = actual;
		}
		else
		{
			cout << "PASA DEL ELSE ?" << endl;
			img_new = actual;

			cout << "[INFO] Calling control law." << endl;
			if (GUO(actual, states[2], matching_results[2]) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			Mat desired_temp, new_points;
			if (Kanade_Lucas_Tomasi(img_old3, img_new, desired_temp, img_points3, states[2], matching_results[2]) < 0)
			{
				cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
			}

			imshow("Frontal camera", img_new);
			imshow("Desired", desired_temp);
			waitKey(1);
		}

		/* string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG3++) + ".jpg";
		imwrite(workspace + saveIMG, img_new);
		cout << "[INFO] << Image saved >>" << saveIMG << endl; */

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[2].img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_results[2].img_matches.cols;
		image_msg->height = matching_results[2].img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_results[2].img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (states[2].initialized)
			cout << "[VELS] Vx: " << states[2].Vx
				  << ", Vy: " << states[2].Vy
				  << ", Vz: " << states[2].Vz
				  << "\nVroll: " << states[2].Vroll
				  << ", Vpitch: " << states[2].Vpitch
				  << ", Wyaw: " << states[2].Vyaw
				  << "\n==> average error: " << matching_results[2].mean_feature_error
				  << "<==" << endl
				  << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
					 msg->encoding.c_str());
	}
}

void imageCallback4(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback function called for drone " << 3 + 1 << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
		cout << "[INFO] Image received" << endl;

		if (contIMG4 < 3)
		{
			contIMG4++;
			cout << "\n[INFO] Detecting keypoints" << endl;

			if (states[3].params.camara != 1)
			{
				cout << "[INFO] Detecting points with ORB" << endl;
				if (Compute_descriptors(actual, img_points4, states[3], matching_results[3]) < 0)
				{
					cout << "[ERROR] No keypoints with matches were found" << endl;
					return;
				}
			}
			else
			{
				cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points4, states[3], matching_results[3]) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
				}
			}
			// cout << "[INFO] img_points4: " << img_points4 << endl;
			cout << "[INFO] matching_result.p1: " << matching_results[3].p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_results[3].p2 << endl;

			img_old4 = actual;
		}
		else
		{
			cout << "PASA DEL ELSE ?" << endl;
			img_new = actual;

			cout << "[INFO] Calling control law." << endl;
			if (GUO(actual, states[3], matching_results[3]) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			Mat desired_temp, new_points;
			if (Kanade_Lucas_Tomasi(img_old4, img_new, desired_temp, img_points4, states[3], matching_results[3]) < 0)
			{
				cout << "[ERROR] Kanade_Lucas_Tomasi failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Kanade_Lucas_Tomasi tracker part has been executed" << endl;
			}

			imshow("Frontal camera", img_new);
			imshow("Desired", desired_temp);
			waitKey(1);
		}

		/* string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG4++) + ".jpg";
		imwrite(workspace + saveIMG, img_new);
		cout << "[INFO] << Image saved >>" << saveIMG << endl; */

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[3].img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_results[3].img_matches.cols;
		image_msg->height = matching_results[3].img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_results[3].img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (states[3].initialized)
			cout << "[VELS] Vx: " << states[3].Vx
				  << ", Vy: " << states[3].Vy
				  << ", Vz: " << states[3].Vz
				  << "\nVroll: " << states[3].Vroll
				  << ", Vpitch: " << states[3].Vpitch
				  << ", Wyaw: " << states[3].Vyaw
				  << "\n==> average error: " << matching_results[3].mean_feature_error
				  << "<==" << endl
				  << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
					 msg->encoding.c_str());
	}
}

void imageCallback1(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback2 function" << endl;

	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
		cout << "[INFO] Image received" << endl;

		imshow("Camera", actual);
		waitKey(1);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void doNothing(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] doNothing function" << endl;
	return;
}

/*
	Function: PoseCallback
	description: get the ppose info from the groundtruth of the drone and uses it in simulation
	params: message with pose info
*/
/* void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	states[ACTUAL_DRON].Roll = (float)roll;
	states[ACTUAL_DRON].Pitch = (float)pitch;

	// setting the position if its the first time
	if (!states[ACTUAL_DRON].initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		states[ACTUAL_DRON].initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}
} */

void poseCallback1(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

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
}

void poseCallback2(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

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
}

void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

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
}

void poseCallback4(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

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
}

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