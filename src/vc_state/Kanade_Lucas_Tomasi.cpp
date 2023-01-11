#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int MinBy(Mat puntos, Mat key);
Mat Orden(Mat puntos);

int Kanade_Lucas_Tomasi(const Mat &actual,
                        Mat &img_points,
                        vc_state &state,
                        vc_homograpy_matching_result &matching_result)
{
   if (compute_descriptors(actual, state.params, state.desired_configuration, matching_result) < 0)
   {
      cout << "[ERROR] Error en compute_descriptors" << endl;
      return -1;
   }

   Mat puntos = Orden(matching_result.p2);
   img_points = Mat(4, 2, CV_32F);
   img_points.at<Point2f>(0, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 0), 0), matching_result.p2.at<double>(puntos.at<int>(0, 0), 1));
   img_points.at<Point2f>(1, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 1), 0), matching_result.p2.at<double>(puntos.at<int>(0, 1), 1));
   img_points.at<Point2f>(2, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 2), 0), matching_result.p2.at<double>(puntos.at<int>(0, 2), 1));
   img_points.at<Point2f>(3, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 3), 0), matching_result.p2.at<double>(puntos.at<int>(0, 3), 1));

   Mat temporal = Mat::zeros(4, 2, CV_32F);
   temporal.at<Point2f>(0, 0) = Point2f(matching_result.p1.at<double>(puntos.at<int>(0, 0), 0), matching_result.p1.at<double>(puntos.at<int>(0, 0), 1));
   temporal.at<Point2f>(1, 0) = Point2f(matching_result.p1.at<double>(puntos.at<int>(0, 1), 0), matching_result.p1.at<double>(puntos.at<int>(0, 1), 1));
   temporal.at<Point2f>(2, 0) = Point2f(matching_result.p1.at<double>(puntos.at<int>(0, 2), 0), matching_result.p1.at<double>(puntos.at<int>(0, 2), 1));
   temporal.at<Point2f>(3, 0) = Point2f(matching_result.p1.at<double>(puntos.at<int>(0, 3), 0), matching_result.p1.at<double>(puntos.at<int>(0, 3), 1));
   temporal.convertTo(matching_result.p1, CV_64F);

   temporal = Mat::zeros(4, 2, CV_32F);
   temporal.at<Point2f>(0, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 0), 0), matching_result.p2.at<double>(puntos.at<int>(0, 0), 1));
   temporal.at<Point2f>(1, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 1), 0), matching_result.p2.at<double>(puntos.at<int>(0, 1), 1));
   temporal.at<Point2f>(2, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 2), 0), matching_result.p2.at<double>(puntos.at<int>(0, 2), 1));
   temporal.at<Point2f>(3, 0) = Point2f(matching_result.p2.at<double>(puntos.at<int>(0, 3), 0), matching_result.p2.at<double>(puntos.at<int>(0, 3), 1));
   temporal.convertTo(matching_result.p2, CV_64F);
   return 0;
}

int MinBy(Mat puntos, Mat key)
{
   // Make buuble sort with norm of the difference between points and key
   Mat orden = Mat::zeros(1, puntos.rows, CV_32S);
   Mat p2 = puntos.clone();

   for (int i = 0; i < p2.rows; i++)
   {
      orden.at<int>(0, i) = i;
   }

   for (int i = 0; i < p2.rows; i++)
   {
      for (int j = 0; j < p2.rows - 1; j++)
      {
         Mat diff1 = p2.row(j) - key;
         Mat diff2 = p2.row(i) - key;
         if (norm(diff1) > norm(diff2))
         {
            double temp = p2.at<double>(j, 0);
            p2.at<double>(j, 0) = p2.at<double>(i, 0);
            p2.at<double>(i, 0) = temp;

            temp = p2.at<double>(j, 1);
            p2.at<double>(j, 1) = p2.at<double>(i, 1);
            p2.at<double>(i, 1) = temp;

            int temp2 = orden.at<int>(0, j);
            orden.at<int>(0, j) = orden.at<int>(0, i);
            orden.at<int>(0, i) = temp2;
         }
      }
   }

   return orden.at<int>(0, 0);
}

Mat Orden(Mat puntos)
{
   Mat orden = Mat::zeros(1, 4, CV_32S);
   // The next are constnat matrices to compare points around about
   Mat key_p1 = (Mat_<double>(1, 2) << 188, 360);
   Mat key_p2 = (Mat_<double>(1, 2) << 564, 360);
   Mat key_p3 = (Mat_<double>(1, 2) << 188, 120);
   Mat key_p4 = (Mat_<double>(1, 2) << 564, 120);

   int mkey_p1 = MinBy(puntos, key_p1);
   int mkey_p2 = MinBy(puntos, key_p2);
   int mkey_p3 = MinBy(puntos, key_p3);
   int mkey_p4 = MinBy(puntos, key_p4);

   orden.at<int>(0, 0) = mkey_p1;
   orden.at<int>(0, 1) = mkey_p2;
   orden.at<int>(0, 2) = mkey_p3;
   orden.at<int>(0, 3) = mkey_p4;

   cout << "Orden: " << orden << endl;

   return orden;
}