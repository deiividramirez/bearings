#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int Kanade_Lucas_Tomasi(const Mat &img_old,
                        const Mat &img_new,
                        Mat &desired_temp,
                        Mat &img_points,
                        vc_state &state,
                        vc_homograpy_matching_result &matching_result)
{ // KLT tracker for the next iteration
   Mat new_points, status, error;
   Mat img_old_gray, img_new_gray;
   cvtColor(img_old, img_old_gray, COLOR_BGR2GRAY);
   cvtColor(img_new, img_new_gray, COLOR_BGR2GRAY);
   calcOpticalFlowPyrLK(img_old_gray, img_new_gray, img_points, new_points, status, error);

   desired_temp = state.desired_configuration.img.clone();
   for (int i = 0; i < matching_result.p1.rows; i++)
   {
      circle(desired_temp, Point2f(matching_result.p1.at<double>(i, 0), matching_result.p1.at<double>(i, 1)), 3, Scalar(0, 0, 255), -1);
   }
   for (int i = 0; i < new_points.rows; i++)
   {
      circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
      circle(img_new, new_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);
      circle(img_new, img_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
   }

   new_points.convertTo(matching_result.p2, CV_64F);
   img_points = new_points.clone();
   img_new.copyTo(img_old);
   return 0;
}
