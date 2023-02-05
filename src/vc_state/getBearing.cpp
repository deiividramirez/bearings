#include "vc_state/vc_state.h"

#include <opencv2/aruco.hpp>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace cv;
using namespace std;

Mat puntoMedio(Mat &p1, Mat &p2, Mat &p3, Mat &p4);
void Tipito(Mat &Matrix);

geometry_msgs::PointStamped pos_msg;
geometry_msgs::PointStamped pos_msg_to;

int getBearing(Mat &actual_image,
               int marker_id,
               Mat &store_bearing,
               Mat &store_ground_truth,
               vc_state &state,
               int drone_id,
               vector<geometry_msgs::PointStamped> &pos_dron)
{

   for (int i = 0; i < pos_dron.size(); i++)
   {
      if (pos_dron[i].header.seq == 0)
      {
         cout << "[INFO] Waiting for position of drone " << i << endl;
         cout << pos_dron[i].header.seq << endl;
         cout << pos_dron[i].point.x << endl;
         cout << pos_dron[i].point.y << endl;
         cout << pos_dron[i].point.z << endl;
         return -1;
      }
   }

   vector<int> markerIds;
   vector<vector<Point2f>> markerCorners, rejectedCandidates;
   Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
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

      store_bearing = Mat::zeros(3, 1, CV_64F);
      store_ground_truth = Mat::zeros(3, 1, CV_64F);

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
      temp = temp2 / norm(temp2);

      store_bearing.at<double>(0, 0) = temp.at<double>(2, 0);
      store_bearing.at<double>(1, 0) = temp.at<double>(0, 0);
      store_bearing.at<double>(2, 0) = temp.at<double>(1, 0);

      // Ground truth
      store_ground_truth.at<double>(0, 0) = pos_dron[marker_id-1].point.x - pos_dron[drone_id-1].point.x;
      store_ground_truth.at<double>(1, 0) = pos_dron[marker_id-1].point.y - pos_dron[drone_id-1].point.y;
      store_ground_truth.at<double>(2, 0) = pos_dron[marker_id-1].point.z - pos_dron[drone_id-1].point.z;

      double norma = norm(store_ground_truth);
      if (norma != 0)
      {
         store_ground_truth = store_ground_truth / norma;
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