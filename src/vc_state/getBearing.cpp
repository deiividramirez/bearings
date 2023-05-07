#include "vc_state/vc_state.h"

#include <opencv2/aruco.hpp>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace cv;
using namespace std;

Mat puntoMedio(Mat &p1, Mat &p2, Mat &p3, Mat &p4);
void Tipito(Mat &Matrix);

geometry_msgs::PoseStamped pos_msg;
geometry_msgs::PoseStamped pos_msg_to;

int getBearing(Mat &actual_image,
               Mat &store_bearing,
               Mat &store_ground_truth,
               vc_state &state,
               int drone_id,
               vector<geometry_msgs::PoseStamped> &pos_dron)
{

   // for (int i = 0; i < pos_dron.size(); i++)
   // {
   //    if (pos_dron[i].header.seq == 0)
   //    {
   //       cout << "[INFO] Waiting for position of drone " << i << endl;
   //       return -1;
   //    }
   // }

   // XmlRpc::XmlRpcValue marker_id;

   store_ground_truth = Mat::zeros(3, state.params.seguimiento.rows, CV_64F);
   Mat grou_temp = Mat::zeros(3, 1, CV_64F);

   for (int32_t marker_index = 0; marker_index < state.params.seguimiento.rows; marker_index++)
   {

      // Ground truth
      grou_temp.at<double>(0, 0) = pos_dron[drone_id - 1].pose.position.x - pos_dron[(int)state.params.seguimiento.at<double>(marker_index, 0) - 1].pose.position.x;
      grou_temp.at<double>(1, 0) = pos_dron[drone_id - 1].pose.position.y - pos_dron[(int)state.params.seguimiento.at<double>(marker_index, 0) - 1].pose.position.y;
      grou_temp.at<double>(2, 0) = pos_dron[drone_id - 1].pose.position.z - pos_dron[(int)state.params.seguimiento.at<double>(marker_index, 0) - 1].pose.position.z;

      double norma = norm(grou_temp);
      if (norma != 0)
      {
         grou_temp = grou_temp / norma;
      }

      store_ground_truth.at<double>(0, marker_index) = grou_temp.at<double>(0, 0);
      store_ground_truth.at<double>(1, marker_index) = grou_temp.at<double>(1, 0);
      store_ground_truth.at<double>(2, marker_index) = grou_temp.at<double>(2, 0);
   }

   vector<int> markerIds;
   vector<vector<Point2f>> markerCorners, rejectedCandidates;
   Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
   try
   {
      aruco::detectMarkers(actual_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
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
      int indice;

      // cout << "[INFO] Called getBearing for drone " << drone_id << endl;

      store_bearing = Mat::zeros(3, state.params.seguimiento.rows, CV_64F);

      for (int32_t marker_index = 0; marker_index < state.params.seguimiento.rows; marker_index++)
      {
         indice = -1;
         for (int i = 0; i < markerIds.size(); i++)
         {
            if (markerIds[i] == (int)state.params.seguimiento.at<double>(marker_index, 0))
            {
               cout << "[INFO] Marker " << state.params.seguimiento.at<double>(marker_index, 0) << " detected" << endl;
               // cout << "[INFO] Marker corners: " << markerCorners[i] << endl;
               indice = i;
               break;
            }
         }

         if (indice == -1)
         {
            cout << "[ERROR] Marker " << state.params.seguimiento.at<double>(marker_index, 0) << " not detected" << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 2, CV_32F);
         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[indice][i].x;
            temporal.at<float>(i, 1) = markerCorners[indice][i].y;
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
         temp = temp2 / norm(temp2);

         Mat bear_temp = Mat::zeros(3, 1, CV_64F);

         bear_temp.at<double>(0, 0) = -temp.at<double>(2, 0);
         bear_temp.at<double>(1, 0) = temp.at<double>(0, 0);
         bear_temp.at<double>(2, 0) = temp.at<double>(1, 0);


         double b1 = bear_temp.at<double>(0, 0);
         double b2 = bear_temp.at<double>(1, 0);

         store_bearing.at<double>(0, marker_index) = cos(state.Yaw) * b1 - sin(state.Yaw) * b2;
         store_bearing.at<double>(1, marker_index) = sin(state.Yaw) * b1 + cos(state.Yaw) * b2;
         // store_bearing.at<double>(0, marker_index) = bear_temp.at<double>(0, 0);
         // store_bearing.at<double>(1, marker_index) = bear_temp.at<double>(1, 0);
         store_bearing.at<double>(2, marker_index) = bear_temp.at<double>(2, 0);
      }
      return 0;
   }
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