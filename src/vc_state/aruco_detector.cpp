#include "vc_state/vc_state.h"
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

int aruco_detector(const Mat &actual,
                   Mat &img_points,
                   vc_state &state,
                   vc_homograpy_matching_result &matching_result,
                   XmlRpc::XmlRpcValue marker_idXLM)
{

   vector<int> markerIds_Detected;
   vector<vector<Point2f>> markerCorners_Detectec, rejectedCandidates;
   Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

   try
   {
      aruco::detectMarkers(actual, dictionary, markerCorners_Detectec, markerIds_Detected, parameters, rejectedCandidates);
   }
   catch (Exception &e)
   {
      cout << "Exception: " << e.what() << endl;
      return -1;
   }

   if (markerCorners_Detectec.size() != 0 && markerIds_Detected.size() != 0)
   {
      cout << "\n[INFO] Markers detected: " << markerIds_Detected.size() << " with marker ids: ";
      for (int i = 0; i < markerIds_Detected.size(); i++)
      {
         cout << markerIds_Detected[i] << " ";
      }
      cout << endl;

      int32_t marker_index = -1;
      for (int32_t indexesXLM = 0; indexesXLM < marker_idXLM.size(); indexesXLM++)
      {

         for (int i = 0; i < markerIds_Detected.size(); i++)
         {
            if (markerIds_Detected[i] == (int)marker_idXLM[indexesXLM])
            {
               cout << "[INFO] Marker " << marker_idXLM[indexesXLM] << " in " << marker_idXLM[indexesXLM] << " detected." << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] All markers in " << marker_idXLM << " not in " << marker_idXLM[indexesXLM] << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners_Detectec[marker_index][0].x, markerCorners_Detectec[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners_Detectec[marker_index][1].x, markerCorners_Detectec[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners_Detectec[marker_index][2].x, markerCorners_Detectec[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners_Detectec[marker_index][3].x, markerCorners_Detectec[marker_index][3].y);
         temporal.convertTo(matching_result.p2, CV_64F);
         temporal.convertTo(img_points, CV_32F);
         temporal.release();

         // if ((int)marker_idXLM[0] == 98 || (int)marker_idXLM[0] == 99)
         // {
         //    cout << "SI ES 98" << endl;
         //    cout << matching_result.p2 << endl;
         //    cout << markerCorners_Detectec[marker_index] << endl;

         //    Mat imageCopy;
         //    actual.copyTo(imageCopy);
         //    // draw just the corners
         //    for (int i = 0; i < markerCorners_Detectec.size(); i++)
         //    {
         //       for (int j = 0; j < markerCorners_Detectec[i].size(); j++)
         //       {
         //          if (i == marker_index)
         //          {
         //             circle(imageCopy, markerCorners_Detectec[i][j], 5, Scalar(0, 0, 255), 2);
         //             cout << "BGR: "
         //                  << "RED " << markerCorners_Detectec[i][j] << endl;
         //          }
         //          else
         //          {
         //             circle(imageCopy, markerCorners_Detectec[i][j], 5, Scalar(0, 255, 0), 2);
         //             cout << "BGR: "
         //                  << "GREEN" << endl;
         //          }
         //       }
         //    }

         //    namedWindow("image", WINDOW_NORMAL);
         //    resizeWindow("image", 960, 540);
         //    imshow("image", imageCopy);
         //    waitKey(1);
         // }

         aruco::detectMarkers(state.desired_configuration.img, dictionary, markerCorners_Detectec, markerIds_Detected, parameters, rejectedCandidates);

         for (int i = 0; i < markerIds_Detected.size(); i++)
         {
            if (markerIds_Detected[i] == (int)marker_idXLM[indexesXLM])
            {
               cout << "[INFO] Marker " << marker_idXLM[indexesXLM] << " in " << marker_idXLM[indexesXLM] << " detected." << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] All markers in " << marker_idXLM << " not in " << marker_idXLM[indexesXLM] << endl;
            return -1;
         }

         temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners_Detectec[marker_index][0].x, markerCorners_Detectec[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners_Detectec[marker_index][1].x, markerCorners_Detectec[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners_Detectec[marker_index][2].x, markerCorners_Detectec[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners_Detectec[marker_index][3].x, markerCorners_Detectec[marker_index][3].y);
         temporal.convertTo(matching_result.p1, CV_64F);
      }
   }
   else
   {
      cout << "[ERROR] No markers detected" << endl;
      return -1;
   }
   return 0;
}