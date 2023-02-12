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

// void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback1(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback2(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback3(const geometry_msgs::Pose::ConstPtr &msg);
void poseCallback4(const geometry_msgs::Pose::ConstPtr &msg);

// void IMGCallback1(const sensor_msgs::Image::ConstPtr &msg);
// void IMGCallback2(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg);
void IMGCallback4(const sensor_msgs::Image::ConstPtr &msg);

void imuCallback1(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback2(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback3(const sensor_msgs::Imu::ConstPtr &msg);
void imuCallback4(const sensor_msgs::Imu::ConstPtr &msg);

Mat convertBearing(XmlRpc::XmlRpcValue bearing, XmlRpc::XmlRpcValue segs);

vector<void (*)(const sensor_msgs::Image::ConstPtr &)> imageCallbacks = {imageCallback, imageCallback2, imageCallback3, imageCallback4};
vector<void (*)(const geometry_msgs::Pose::ConstPtr &)> posesCallback = {poseCallback1, poseCallback2, poseCallback3, poseCallback4};
// vector<void (*)(const sensor_msgs::Image::ConstPtr &)> IMGCallbacks = {IMGCallback1, IMGCallback2, IMGCallback3, IMGCallback4};
vector<void (*)(const sensor_msgs::Imu::ConstPtr &)> imuCallbacks = {imuCallback1, imuCallback2, imuCallback3, imuCallback4};

geometry_msgs::PoseStamped pos_dron1, pos_dron2, pos_dron3, pos_dron4;
vector<geometry_msgs::PoseStamped> pos_dron = {pos_dron1, pos_dron2, pos_dron3, pos_dron4};

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
int SAVE_IMAGES, SAVE_DESIRED_IMAGES, SHOW_IMAGES;

double Kp, Kv;

XmlRpc::XmlRpcValue seg1, seg2, seg3, seg4;
vector<XmlRpc::XmlRpcValue> segmentsXML = {seg1, seg2, seg3, seg4};

XmlRpc::XmlRpcValue bearing1, bearing2, bearing3, bearing4;
vector<XmlRpc::XmlRpcValue> bearingsXML = {bearing1, bearing2, bearing3, bearing4};

Mat bearing1_points, bearing2_points, bearing3_points, bearing4_points;
vector<Mat> bearings = {bearing1_points, bearing2_points, bearing3_points, bearing4_points};

bool target1, target2, target3, target4;
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

	gen.getParam("Kp", Kp);
	gen.getParam("Kv", Kv);

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
		image_sub_1b = it1.subscribe("/iris_1/camera_under_camera/image_raw", 1, saveDesired1b);

		image_sub_2f = it2.subscribe("/iris_2/camera_front_camera/image_raw", 1, saveDesired2f);
		image_sub_2b = it2.subscribe("/iris_2/camera_under_camera/image_raw", 1, saveDesired2b);

		image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, saveDesired3f);
		image_sub_3b = it3.subscribe("/iris_3/camera_under_camera/image_raw", 1, saveDesired3b);

		image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, saveDesired4f);
		image_sub_4b = it4.subscribe("/iris_4/camera_under_camera/image_raw", 1, saveDesired4b);
	}
	else
	{
		/****************** FOR CONTROL LAW ******************/
		image_sub_1f = it1.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback);
		// image_sub_1f = it1.subscribe("/iris_1/camera_front_camera/image_raw", 1, doNothing);
		// image_sub_1b = it1.subscribe("/iris_1/camera_under_camera/image_raw", 1, doNothing);

		image_sub_2f = it2.subscribe("/iris_2/camera_front_camera/image_raw", 1, imageCallback2);
		// image_sub_2f = it2.subscribe("/iris_2/camera_front_camera/image_raw", 1, doNothing);
		// image_sub_2b = it2.subscribe("/iris_2/camera_under_camera/image_raw", 1, doNothing);

		image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, IMGCallback3);
		// image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, imageCallback3);
		// image_sub_3f = it3.subscribe("/iris_3/camera_front_camera/image_raw", 1, doNothing);
		// image_sub_3b = it3.subscribe("/iris_3/camera_under_camera/image_raw", 1, doNothing);

		image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, IMGCallback4);
		// image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, imageCallback4);
		// image_sub_4f = it4.subscribe("/iris_4/camera_front_camera/image_raw", 1, doNothing);
		// image_sub_4b = it4.subscribe("/iris_4/camera_under_camera/image_raw", 1, doNothing);
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
		// for to compute points in image
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

		// for to suscribe and advertise
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
			cout << "\n[INFO] All the drones have been initialized." << endl;

			/****************** SAVE DATA ******************/

			// for to save data in array and check if the drone is in the target
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

					X[i].push_back(pos_dron[i].pose.position.x);
					Y[i].push_back(pos_dron[i].pose.position.y);
					Z[i].push_back(pos_dron[i].pose.position.z);

					if (matching_results[i].mean_feature_error < states[i].params.feature_threshold)
					{
						cout << "\n[INFO] Target reached within the feature threshold for drone " + to_string(i + 1) << endl;
						states[i].in_target = true;
						/* if (SHOW_IMAGES)
						{
							waitKey(0);
						} */
						break;
					}
				}
				else
				{
					cout << "\n[INFO] Target reached for drone" + to_string(i + 1) << endl;
					/* if (SHOW_IMAGES)
					{
						waitKey(0);
					} */
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
				rate.sleep();
			}
		}

		if (contIMG1 > 1000 || contIMG2 > 1000 || contIMG3 > 1000 || contIMG4 > 1000 || contGEN > 1000)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			/* break; */
		}
		else
		{
			contGEN++;
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
		writeFile(X[i], workspace + file_folder + "out_X_" + to_string(i + 1) + ".txt");
		writeFile(Y[i], workspace + file_folder + "out_Y_" + to_string(i + 1) + ".txt");
		writeFile(Z[i], workspace + file_folder + "out_Z_" + to_string(i + 1) + ".txt");
	}

	return 0;
}

void IMGCallback3(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << endl
		  << "--------------------> EMPIEZA <--------------------" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

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
		cout << "Bearing with ground truth drone " << 2 + 1 << endl;
		cout << bearing_ground_truth << endl;

		cout << "bearings[2] = " << bearings[2] << endl;
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
		}
	}

	if (SAVE_IMAGES)
	{
		string saveIMG = "/src/bearings/src/data/img/3_" + to_string(contIMG3++) + ".jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;
	}
	else
	{
		string saveIMG = "/src/bearings/src/data/img/3_1.jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;
	}
}
void IMGCallback4(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << endl
		  << "--------------------> EMPIEZA <--------------------" << endl;
	Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;

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
		cout << "Bearing with ground truth drone " << 3 + 1 << endl;
		cout << bearing_ground_truth << endl;

		cout << "bearings[3] = " << bearings[3] << endl;
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
		}
	}

	if (SAVE_IMAGES)
	{
		string saveIMG = "/src/bearings/src/data/img/4_" + to_string(contIMG4++) + ".jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;
	}
	else
	{
		string saveIMG = "/src/bearings/src/data/img/4_1.jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;
	}
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	if (states[0].initialized)
	{
		cout << endl
			  << "--------------------> EMPIEZA <--------------------" << endl;
		cout << "\n[INFO] ImageCallback function called for drone " << 0 + 1 << endl;

		try
		{
			Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
			cout << "[INFO] Image received" << endl;

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
				img_new = actual;
			}
			else
			{
				img_new = actual;
				if (aruco_detector(actual, img_points1, states[0], matching_results[0], seg1) == 0)
				{
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
				// if (Kanade_Lucas_Tomasi(img_old1, img_new, desired_temp, img_points1, states[0], matching_results[0]) < 0)
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
				// 	imshow("Frontal camera", img_new);
				// 	imshow("Desired", desired_temp);
				// 	waitKey(1);
				// }
			}

			if (SAVE_IMAGES)
			{
				string saveIMG = "/src/bearings/src/data/img/1_" + to_string(contIMG1++) + ".jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}
			else
			{
				string saveIMG = "/src/bearings/src/data/img/1_1.jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
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
}
void imageCallback2(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << endl
		  << "--------------------> EMPIEZA <--------------------" << endl;
	cout << "\n[INFO] ImageCallback function called for drone " << 1 + 1 << endl;

	if (states[1].initialized)
	{
		try
		{
			Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
			cout << "[INFO] Image received" << endl;

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
				img_new = actual;
			}
			else
			{
				img_new = actual;

				cout << "[INFO] Detecting points with ArUco" << endl;
				if (aruco_detector(actual, img_points2, states[1], matching_results[1], seg2) == 0)
				{
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
				}
				else
				{
					cout << "[ERROR] No ArUco were found." << endl;
					// ros::shutdown();
				}

				// Mat desired_temp, new_points;
				// if (Kanade_Lucas_Tomasi(img_old2, img_new, desired_temp, img_points2, states[1], matching_results[1]) < 0)
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
				// 	imshow("Frontal camera_2", img_new);
				// 	waitKey(0);
				// }
			}

			if (SAVE_IMAGES)
			{
				string saveIMG = "/src/bearings/src/data/img/2_" + to_string(contIMG2++) + ".jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}
			else
			{
				string saveIMG = "/src/bearings/src/data/img/2_1.jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
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
}
void imageCallback3(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "\n[INFO] ImageCallback function called for drone " << 2 + 1 << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image, img_new;
		cout << "[INFO] Image received" << endl;

		if (contIMG3++ > 4)
		{
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
				if (aruco_detector(actual, img_points3, states[2], matching_results[2], seg3) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
					return;
				}
			}
			// cout << "[INFO] img_points3: " << img_points3 << endl;
			cout << "[INFO] matching_result.p1: " << matching_results[2].p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_results[2].p2 << endl;

			img_old3 = actual;
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
			if (SHOW_IMAGES)
			{
				imshow("Frontal camera", img_new);
				imshow("Desired", desired_temp);
				waitKey(1);
			}

			if (SAVE_IMAGES)
			{
				string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG3++) + ".jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}

			/************************************************************* Prepare message */
			image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[2].img_matches).toImageMsg();
			image_msg->header.frame_id = "matching_image";
			image_msg->width = matching_results[2].img_matches.cols;
			image_msg->height = matching_results[2].img_matches.rows;
			image_msg->is_bigendian = false;
			image_msg->step = sizeof(unsigned char) * matching_results[2].img_matches.cols * 3;
			image_msg->header.stamp = ros::Time::now();

			/* cout << "[INFO] Matching published" << endl; */

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

		if (contIMG4++ > 4)
		{
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
				if (aruco_detector(actual, img_points4, states[3], matching_results[3], seg4) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					ros::shutdown();
				}
			}
			// cout << "[INFO] img_points4: " << img_points4 << endl;
			cout << "[INFO] matching_result.p1: " << matching_results[3].p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_results[3].p2 << endl;

			img_old4 = actual;
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

			if (SHOW_IMAGES)
			{
				imshow("Frontal camera", img_new);
				imshow("Desired", desired_temp);
				waitKey(1);
			}

			if (SAVE_IMAGES)
			{
				string saveIMG = "/src/bearings/src/data/img/" + to_string(contIMG4++) + ".jpg";
				imwrite(workspace + saveIMG, img_new);
				cout << "[INFO] << Image saved >>" << saveIMG << endl;
			}

			/************************************************************* Prepare message */
			image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_results[3].img_matches).toImageMsg();
			image_msg->header.frame_id = "matching_image";
			image_msg->width = matching_results[3].img_matches.cols;
			image_msg->height = matching_results[3].img_matches.rows;
			image_msg->is_bigendian = false;
			image_msg->step = sizeof(unsigned char) * matching_results[3].img_matches.cols * 3;
			image_msg->header.stamp = ros::Time::now();

			/* cout << "[INFO] Matching published" << endl; */

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
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
					 msg->encoding.c_str());
	}
}

void doNothing(const sensor_msgs::Image::ConstPtr &msg)
{
	/* cout << "\n[INFO] doNothing function" << endl; */
	return;
}

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