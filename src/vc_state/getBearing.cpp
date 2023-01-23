#include "vc_state/vc_state.h"

#include <opencv2/aruco.hpp>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace cv;
using namespace std;

void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
void positionCallback2(const geometry_msgs::PointStamped::ConstPtr &msg);
Mat puntoMedio(Mat &p1, Mat &p2, Mat &p3, Mat &p4);
void Tipito(Mat &Matrix);

geometry_msgs::PointStamped pos_msg;
geometry_msgs::PointStamped pos_msg_to;

int getBearing(Mat &actual,
               int marker_id,
               Mat &bearing,
               Mat &ground_truth,
               vc_state &state,
               int drone_id)
{

   vector<int> markerIds;
   vector<vector<Point2f>> markerCorners, rejectedCandidates;
   Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
   try
   {
      aruco::detectMarkers(actual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
   }
   catch (Exception &e)
   {
      cout << "Exception: " << e.what() << endl;
      return -1;
   }

   if (markerCorners.size() == 0 || markerIds.size() == 0)
   {
      cout << "[ERROR] No markers detected" << endl;
      return -1;
   }
   else
   {

      int marker_index = -1;
      for (int i = 0; i < markerIds.size(); i++)
      {
         if (markerIds[i] == marker_id)
         {
            cout << "[INFO] Marker " << marker_id << " detected" << endl;
            /* cout << "[INFO] Marker corners: " << markerCorners[i] << endl; */
            marker_index = i;
            break;
         }
      }

      if (marker_index == -1)
      {
         cout << "[ERROR] Marker " << marker_id << " not detected" << endl;
         return -1;
      }

      bearing = Mat::zeros(3, 1, CV_64F);
      ground_truth = Mat::zeros(3, 1, CV_64F);

      Mat temporal = Mat::zeros(4, 2, CV_32F);
      for (int i = 0; i < 4; i++)
      {
         temporal.at<float>(i, 0) = markerCorners[marker_index][i].x;
         temporal.at<float>(i, 1) = markerCorners[marker_index][i].y;
      }

      temporal.convertTo(temporal, CV_64F);

      Mat p1 = temporal.row(0);
      Mat p2 = temporal.row(1);
      Mat p3 = temporal.row(2);
      Mat p4 = temporal.row(3);

      Mat pMedio = puntoMedio(p1, p2, p3, p4);

      Mat temp = Mat(3, 1, CV_64F);
      temp.at<double>(0, 0) = pMedio.at<double>(0, 0);
      temp.at<double>(1, 0) = pMedio.at<double>(0, 1);
      temp.at<double>(2, 0) = 1;

      Mat temp2 = state.params.K.inv() * temp;
      temp = -temp2 / norm(temp2);

      bearing.at<double>(0, 0) = temp.at<double>(2, 0);
      bearing.at<double>(1, 0) = temp.at<double>(0, 0);
      bearing.at<double>(2, 0) = temp.at<double>(1, 0);

      ros::NodeHandle nh;
      string topic_sub = "/iris_" + to_string(drone_id) + "/ground_truth/position";
      string topic_sub_to = "/iris_" + to_string(marker_id) + "/ground_truth/position";

      ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PointStamped>(topic_sub, 1, positionCallback, ros::TransportHints().tcpNoDelay());
      ros::Subscriber pos_sub_to = nh.subscribe<geometry_msgs::PointStamped>(topic_sub_to, 1, positionCallback2, ros::TransportHints().tcpNoDelay());

      bool flag = false;
      bool p1bool = (pos_msg.point.x != 0 || pos_msg.point.y != 0 || pos_msg.point.z != 0);
      bool p2bool = (pos_msg_to.point.x != 0 || pos_msg_to.point.y != 0 || pos_msg_to.point.z != 0);

      cout << "P1: " << (p1bool ? "true" : "false") << " - " << pos_msg.point << endl;
      cout << "P2: " << (p2bool ? "true" : "false") << " - " << pos_msg_to.point << endl;

      while (!flag && !p1bool && !p2bool)
      {
         try
         {
            ground_truth.at<double>(0, 0) = pos_msg_to.point.x - pos_msg.point.x;
            ground_truth.at<double>(1, 0) = pos_msg_to.point.y - pos_msg.point.y;
            ground_truth.at<double>(2, 0) = pos_msg_to.point.z - pos_msg.point.z;

            if (ground_truth.at<double>(0, 0) != 0 || ground_truth.at<double>(1, 0) != 0 || ground_truth.at<double>(2, 0) != 0)
            {
               flag = true;
               ground_truth = ground_truth / norm(ground_truth);
               /* cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
               cout << "[INFO] Ground truth: " << ground_truth << endl;
               cout << "[INFO] Position: " << pos_msg << endl;
               cout << "[INFO] Position to: " << pos_msg_to << endl;
               cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl; */
               return 0;
            }
         }
         catch (Exception &e)
         {
            cout << "Exception: " << e.what() << endl;
         }
         ros::spinOnce();
      }

      return 0;
   }
}

void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
   /* cout << "[INFO] PositionCallback" << endl; */
   pos_msg.point.x = msg->point.x;
   pos_msg.point.y = msg->point.y;
   pos_msg.point.z = msg->point.z;
   /* cout << "[INFO] Position: " << pos_msg << endl; */
}

void positionCallback2(const geometry_msgs::PointStamped::ConstPtr &msg)
{
   /* cout << "[INFO] PositionCallback2" << endl; */
   pos_msg_to.point.x = msg->point.x;
   pos_msg_to.point.y = msg->point.y;
   pos_msg_to.point.z = msg->point.z;
   /* cout << "[INFO] Position to: " << pos_msg_to << endl; */
}

Mat puntoMedio(Mat &p1, Mat &p2, Mat &p3, Mat &p4)
{
   Mat pMedio = Mat::zeros(1, 2, CV_64F);
   pMedio.at<double>(0, 0) = (p1.at<double>(0, 0) + p2.at<double>(0, 0) + p3.at<double>(0, 0) + p4.at<double>(0, 0)) / 4;
   pMedio.at<double>(0, 1) = (p1.at<double>(0, 1) + p2.at<double>(0, 1) + p3.at<double>(0, 1) + p4.at<double>(0, 1)) / 4;
   return pMedio;
}

string type2str(int type)
{
   string r;

   uchar depth = type & CV_MAT_DEPTH_MASK;
   uchar chans = 1 + (type >> CV_CN_SHIFT);

   switch (depth)
   {
   case CV_8U:
      r = "8U";
      break;
   case CV_8S:
      r = "8S";
      break;
   case CV_16U:
      r = "16U";
      break;
   case CV_16S:
      r = "16S";
      break;
   case CV_32S:
      r = "32S";
      break;
   case CV_32F:
      r = "32F";
      break;
   case CV_64F:
      r = "64F";
      break;
   default:
      r = "User";
      break;
   }

   r += "C";
   r += (chans + '0');

   return r;
}

void Tipito(Mat &Matrix)
{
   string ty = type2str(Matrix.type());
   cout << "Matrix: " << ty.c_str() << " " << Matrix.cols << "x" << Matrix.rows << endl;
   cout << Matrix << endl;
}