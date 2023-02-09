#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

Mat projOrtog(Mat &x);

int bearingControl(Mat actual_bearing, 
                   Mat position,
                   Mat desired_bearings,
                   vector<vc_state> &states,
                   XmlRpc::XmlRpcValue &marker_ids,
                   int drone_id,
                   double Kp, 
                   double Kv)
{
   
   Mat suma = Mat::zeros(3, 1, CV_64F), temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);
   for (int32_t i = 0; i < marker_ids.size(); i++)
   {
      temp = desired_bearings.col(i);
      temp2.at<double>(0, 0) = states[drone_id-1].Vx - states[(int)marker_ids[i]-1].Vx;
      temp2.at<double>(1, 0) = states[drone_id-1].Vy - states[(int)marker_ids[i]-1].Vy;
      temp2.at<double>(2, 0) = states[drone_id-1].Vz - states[(int)marker_ids[i]-1].Vz;


      suma = suma - projOrtog(temp) * (Kp * position.col(i) + Kv * temp2);
   }

   states[drone_id-1].Vx = suma.at<double>(0, 0);
   states[drone_id-1].Vy = suma.at<double>(1, 0);
   states[drone_id-1].Vz = suma.at<double>(2, 0);

   cout << ">>>> [ " << states[drone_id-1].Vx << " " << states[drone_id-1].Vy << " " << states[drone_id-1].Vz << " ]<<<<" << endl;

   return 0;
}

Mat projOrtog(Mat &x)
{
   Mat Px = (Mat::eye(3, 3, CV_64F) - x * x.t());
   return Px;
}