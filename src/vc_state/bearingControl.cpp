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

   Mat suma1 = Mat::zeros(3, 1, CV_64F), temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);
   Mat suma2 = Mat::zeros(3, 1, CV_64F);
   for (int32_t i = 0; i < marker_ids.size(); i++)
   {
      int opc = states[drone_id - 1].params.control;
      if (opc == 0)
      {
         // Control with position
         temp = desired_bearings.col(i);
         temp2.at<double>(0, 0) = states[drone_id - 1].Vx - states[(int)marker_ids[i] - 1].Vx;
         temp2.at<double>(1, 0) = states[drone_id - 1].Vy - states[(int)marker_ids[i] - 1].Vy;
         temp2.at<double>(2, 0) = states[drone_id - 1].Vz - states[(int)marker_ids[i] - 1].Vz;
         suma1 -= projOrtog(temp) * (Kp * position.col(i) + Kv * temp2);
      }
      else if (opc == 1)
      {
         // Control with bearing
         suma1 += (-actual_bearing.col(i) - desired_bearings.col(i));
         states[drone_id -1].integral_error += states[drone_id - 1].dt * suma1;
      }
      else if (opc == 2)
      {
         // Control with difference of bearing
         temp = actual_bearing.col(i);
         suma2 -= projOrtog(temp) * (desired_bearings.col(i));

         states[drone_id -1].integral_error += states[drone_id - 1].dt * suma2;
      }
      else if (opc == 3)
      {
         // Control with difference of bearings and orthogonal projection
         temp = actual_bearing.col(i);
         suma1 += -actual_bearing.col(i) - desired_bearings.col(i);
         suma2 -= projOrtog(temp) * (desired_bearings.col(i));

         states[drone_id -1].integral_error += states[drone_id - 1].dt * suma1;
      }
   }

   cout << "Integral error: " << states[drone_id - 1].integral_error << endl;

   // Error calculation
   states[drone_id - 1].error = norm(desired_bearings + actual_bearing);

   // Variable lambda - gain
   double l0_Kp = 4 * Kp, linf_Kp = Kp, lprima_Kp = 1;
   double lambda_Kp = (l0_Kp - linf_Kp) * exp(-(lprima_Kp * states[drone_id - 1].error) / (l0_Kp - linf_Kp)) + linf_Kp;

   double l0_Kv = 4 * Kv, linf_Kv = Kv, lprima_Kv = 1;
   double lambda_Kv = (l0_Kv - linf_Kv) * exp(-(lprima_Kv * states[drone_id - 1].error) / (l0_Kv - linf_Kv)) + linf_Kv;

   cout << endl
        << "[INFO] Lambda: " << linf_Kp << " < " << lambda_Kp << " < " << l0_Kp << endl
        << "[INFO] Lambda: " << linf_Kv << " < " << lambda_Kp << " < " << l0_Kv << endl;

   // lambda_Kp = 3;
   states[drone_id - 1].lambda = lambda_Kp;

   // Update the velocity
   Mat suma3 = suma1 + suma2;
   states[drone_id - 1].Vx = lambda_Kp * suma3.at<double>(0, 0) + lambda_Kv * states[drone_id - 1].integral_error.at<double>(0, 0);
   states[drone_id - 1].Vy = lambda_Kp * suma3.at<double>(1, 0) + lambda_Kv * states[drone_id - 1].integral_error.at<double>(1, 0);
   states[drone_id - 1].Vz = lambda_Kp * suma3.at<double>(2, 0) + lambda_Kv * states[drone_id - 1].integral_error.at<double>(2, 0);

   cout << "Desired bearing: " << desired_bearings << endl;
   cout << "Actual bearing: " << actual_bearing << endl;

   // cout << ">>>> [ " << states[drone_id - 1].Vx << " " << states[drone_id - 1].Vy << " " << states[drone_id - 1].Vz << " ]<<<<" << endl;

   return 0;
}

Mat projOrtog(Mat &x)
{
   Mat Px = (Mat::eye(3, 3, CV_64F) - x * x.t());
   return Px;
}