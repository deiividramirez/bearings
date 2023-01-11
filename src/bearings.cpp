#include "bearings.h"

/****************** DECLARING NAMESPACES ******************/
using namespace cv;
using namespace std;

/****************** CALLBACKS ******************/
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void imageCallback2(const sensor_msgs::Image::ConstPtr &msg);
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void writeFile(vector<float> &vec, const string &name);

/****************** DECLARING OBJECTS TO RECEIVE MESSAGES ******************/
sensor_msgs::ImagePtr image_msg;

/****************** WORKSPACE DEFINITION FROM CMAKE ******************/
string workspace = WORKSPACE;

/****************** VISUAL CONTROL STATE AND RESULTS VARIABLES ******************/
vc_state state;
vc_homograpy_matching_result matching_result;

/****************** AUXILIAR GLOBAL VARIABLES ******************/
int contIMG = 0;
bool cam3 = false;
Mat img_old, img_points;

/****************** MAIN FUNCTION ******************/
int main(int argc, char **argv)
{
	/****************** ROS INITIALIZING ******************/
	ros::init(argc, argv, "bearings");
	ros::NodeHandle nh;
	state.load(nh);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub_1f, image_sub_1b, 
										 image_sub_2f, image_sub_2b, 
										 image_sub_3f, image_sub_3b, 
										 image_sub_4f, image_sub_4b;

	/****************** CREATING PUBLISHER AND SUBSCRIBER ******************/
	string image_dir = "/src/bearings/src/desired2.jpg";

	image_sub_1f = it.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback2);
	image_sub_1b = it.subscribe("/iris_1/camera_under_camera/image_raw", 1, imageCallback2);

	image_sub_2f = it.subscribe("/iris_2/camera_front_camera/image_raw", 1, imageCallback2);
	image_sub_2b = it.subscribe("/iris_2/camera_under_camera/image_raw", 1, imageCallback2);

	image_sub_3f = it.subscribe("/iris_3/camera_front_camera/image_raw", 1, imageCallback2);
	image_sub_3b = it.subscribe("/iris_3/camera_under_camera/image_raw", 1, imageCallback2);

	image_sub_4f = it.subscribe("/iris_4/camera_front_camera/image_raw", 1, imageCallback2);
	image_sub_4b = it.subscribe("/iris_4/camera_under_camera/image_raw", 1, imageCallback2);

	// if (state.params.camara == 0)
	// {
	// 	cout << "[INFO] Using hummingbird bottom camera" << endl;
	// 	image_sub_1f = it.subscribe("/hummingbird/camera_nadir/image_raw", 1, imageCallback);
	// 	image_dir = "/src/bearings/src/desired.jpg";
	// }
	// else if (state.params.camara == 1)
	// {
	// 	cout << "[INFO] Using iris front camera" << endl;
	// 	image_sub_1f = it.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback);
	// 	image_dir = "/src/bearings/src/desired2.jpg";
	// }
	// else if (state.params.camara == 2)
	// {
	// 	cout << "[INFO] Using iris bottom camera" << endl;
	// 	image_sub_1f = it.subscribe("/iris_1/camera_under_camera/image_raw", 1, imageCallback);
	// 	image_dir = "/src/bearings/src/desired.jpg";
	// }
	// else if (state.params.camara == 3)
	// {
	// 	state.params.camara = 1;
	// 	cout << "[INFO] Using both cameras" << endl;
	// 	image_sub_1f = it.subscribe("/iris_1/camera_front_camera/image_raw", 1, imageCallback);
	// 	image_sub2 = it.subscribe("/iris_1/camera_under_camera/image_raw", 1, imageCallback2);
	// 	image_dir = "/src/bearings/src/desired2.jpg";
	// 	cam3 = true;
	// }
	// else
	// {
	// 	cout << "[ERROR] There is no camera with that number" << endl;
	// 	exit(-1);
	// }

	// image_transport::Publisher image_pub = it.advertise("matching", 1);
	ros::Rate rate(30);
	// ros::Rate rate(120);

	/************************************************************************** OPENING DESIRED IMAGE */

	state.desired_configuration.img = imread(workspace + image_dir, IMREAD_COLOR);
	if (state.desired_configuration.img.empty())
	{
		cerr << "[ERROR] Could not open or find the reference image" << std::endl;
		cout << "[ERROR] No dir >> " << workspace + image_dir << endl;
		exit(-1);
	}
	else
	{
		cout << "[INFO] Reference image loaded" << std::endl;
	}

	Ptr<ORB> orb = ORB::create(state.params.nfeatures,
										state.params.scaleFactor,
										state.params.nlevels,
										state.params.edgeThreshold,
										state.params.firstLevel,
										state.params.WTA_K,
										state.params.scoreType,
										state.params.patchSize,
										state.params.fastThreshold);

	orb->detect(state.desired_configuration.img,
					state.desired_configuration.kp);
	orb->compute(state.desired_configuration.img,
					 state.desired_configuration.kp,
					 state.desired_configuration.descriptors);

	/******************************************************************************* MOVING TO A POSE */
	ros::Publisher pos_pub;
	ros::Subscriber pos_sub;

	if (state.params.camara == 0)
	{
		cout << "[INFO] Hummingbird trajectory and pose" << endl
			  << endl;
		pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 1);
		pos_sub = nh.subscribe<geometry_msgs::Pose>("/hummingbird/ground_truth/pose", 1, poseCallback);
	}
	else if (state.params.camara == 1 || state.params.camara == 2)
	{
		cout << "[INFO] Iris trajectory and pose" << endl
			  << endl;
		pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/iris_1/command/trajectory", 1);
		pos_sub = nh.subscribe<geometry_msgs::Pose>("/iris_1/ground_truth/pose", 1, poseCallback);
	}

	/**************************************************************************** data for graphics */
	vector<float> vel_x;
	vector<float> vel_y;
	vector<float> vel_z;
	vector<float> vel_yaw;
	vector<float> errors;
	vector<float> errors_pix;
	vector<float> time;
	vector<float> lambda;

	// Create message for the pose
	trajectory_msgs::MultiDOFJointTrajectory msg;
	string file_folder = "/src/bearings/src/data/";

	/******************************************************************************* CYCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();

		if (!state.initialized)
		{
			rate.sleep();
			continue;
		} // if we havent get the new pose

		// save data
		time.push_back(state.t);
		errors.push_back((float)matching_result.mean_feature_error);
		errors_pix.push_back((float)matching_result.mean_feature_error_pix);
		vel_x.push_back(state.Vx);
		vel_y.push_back(state.Vy);
		vel_z.push_back(state.Vz);
		vel_yaw.push_back(state.Vyaw);
		lambda.push_back(state.lambda);

		if (matching_result.mean_feature_error < state.params.feature_threshold)
		{
			cout << endl
				  << "[INFO] Target reached within the feature threshold and maximum iterations" << endl
				  << endl;
			waitKey(0);
			break;
		}

		// Publish image of the matching
		cout << endl
			  << "[INFO] Publishing image" << endl;
		// image_pub.publish(image_msg);

		// Update state with the current control
		auto new_pose = state.update();

		// Prepare msg
		msg.header.stamp = ros::Time::now();
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

		// Publish msg
		pos_pub.publish(msg);
		rate.sleep();

		if (contIMG > 1000)
		{
			cout << "[ERROR] No convergence, quitting" << endl;
			break;
		}
	}

	// save data
	writeFile(errors, workspace + file_folder + "errors.txt");
	writeFile(errors_pix, workspace + file_folder + "errors_pix.txt");
	writeFile(time, workspace + file_folder + "time.txt");
	writeFile(vel_x, workspace + file_folder + "Vx.txt");
	writeFile(vel_y, workspace + file_folder + "Vy.txt");
	writeFile(vel_z, workspace + file_folder + "Vz.txt");
	writeFile(vel_yaw, workspace + file_folder + "Vyaw.txt");
	writeFile(lambda, workspace + file_folder + "lambda.txt");

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
	cout << "[INFO] ImageCallback function" << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
		cout << "[INFO] Image received" << endl;

		if (contIMG < 3)
		{
			contIMG++;
			cout << "\n[INFO] Detecting keypoints" << endl;

			if (state.params.camara != 1)
			{
				if (Kanade_Lucas_Tomasi(actual, img_points, state, matching_result) < 0)
				{
					cout << "[ERROR] No Kanade-Lucas-Tomasi tracking were possible" << endl;
					return;
				}
			}
			else
			{
				if (aruco_detector(actual, img_points, state, matching_result) < 0)
				{
					cout << "[ERROR] No ArUco were found." << endl;
					return;
				}
			}
			// cout << "[INFO] img_points: " << img_points << endl;
			cout << "[INFO] matching_result.p1: " << matching_result.p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_result.p2 << endl;

			img_old = actual;
		}
		else
		{
			Mat img_new = actual;

			// AQUI ES DONDE SE EJECUTA TODOOOOO
			if (GUO(actual, state, matching_result) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			// KLT tracker for the next iteration
			Mat new_points, status, error;
			Mat img_old_gray, img_new_gray;
			cvtColor(img_old, img_old_gray, COLOR_BGR2GRAY);
			cvtColor(img_new, img_new_gray, COLOR_BGR2GRAY);
			calcOpticalFlowPyrLK(img_old_gray, img_new_gray, img_points, new_points, status, error);

			Mat desired_temp = state.desired_configuration.img.clone();
			for (int i = 0; i < matching_result.p1.rows; i++)
			{
				circle(desired_temp, Point2f(matching_result.p1.at<double>(i, 0), matching_result.p1.at<double>(i, 1)), 3, Scalar(0, 0, 255), -1);
			}
			for (int i = 0; i < new_points.rows; i++)
			{
				circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
				circle(actual, new_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);
				circle(actual, img_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
			}

			imshow("Frontal camera", actual);
			imshow("Desired", desired_temp);
			waitKey(1);

			new_points.convertTo(matching_result.p2, CV_64F);
			img_points = new_points.clone();
			actual.copyTo(img_old);
		}

		string saveIMG = "/src/Bearings/src/data/img/" + to_string(contIMG++) + ".jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;

		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_result.img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_result.img_matches.cols;
		image_msg->height = matching_result.img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_result.img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (state.initialized)
			cout << "[VELS] Vx: " << state.Vx << ", Vy: " << state.Vy << ", Vz: " << state.Vz << "\nVroll: " << state.Vroll << ", Vpitch: " << state.Vpitch << ", Wyaw: " << state.Vyaw << "\n==> average error: " << matching_result.mean_feature_error << "<==" << endl
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
	cout << "[INFO] ImageCallback function" << endl;
	if (cam3)
	{
		try
		{
			Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
			cout << "[INFO] Image received" << endl;

			imshow("Camera", actual);
			waitKey(1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
						 msg->encoding.c_str());
		}
	}
}

/*
	Function: PoseCallback
	description: get the ppose info from the groundtruth of the drone and uses it in simulation
	params: message with pose info
*/
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
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
	state.Roll = (float)roll;
	state.Pitch = (float)pitch;

	// setting the position if its the first time
	if (!state.initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		state.initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
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