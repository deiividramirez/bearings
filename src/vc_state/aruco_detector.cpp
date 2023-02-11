#include "vc_state/vc_state.h"
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

int aruco_detector(const Mat &actual,
                   Mat &img_points,
                   vc_state &state,
                   vc_homograpy_matching_result &matching_result,
                   XmlRpc::XmlRpcValue marker_id)
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
      cout << "\n[INFO] Markers detected: " << markerIds.size() << endl;
      cout << "[INFO] Marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }

      cout << endl;

      int marker_index = -1;

      for (int32_t marker_index = 0; marker_index < markerIds.size(); marker_index++)
      {

         for (int i = 0; i < markerIds.size(); i++)
         {
            if (markerIds[i] == (int)marker_id[marker_index])
            {
               cout << "[INFO] Marker " << marker_id << " detected" << endl;
               cout << "[INFO] Marker corners: " << markerCorners[i] << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] Marker " << marker_id << " not detected" << endl;
            return -1;
         }
         
         ros::Duration(0.5).sleep();

         Mat temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners[marker_index][0].x, markerCorners[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners[marker_index][1].x, markerCorners[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners[marker_index][2].x, markerCorners[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners[marker_index][3].x, markerCorners[marker_index][3].y);
         temporal.convertTo(matching_result.p2, CV_64F);
         temporal.convertTo(img_points, CV_32F);

         aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
         temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners[marker_index][0].x, markerCorners[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners[marker_index][1].x, markerCorners[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners[marker_index][2].x, markerCorners[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners[marker_index][3].x, markerCorners[marker_index][3].y);
         temporal.convertTo(matching_result.p1, CV_64F);
      }
   }
   return 0;
}