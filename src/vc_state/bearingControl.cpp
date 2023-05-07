#include "vc_state/vc_state.h"

Mat projOrtog(Mat &x);

int bearingControl(Mat actual_bearing,
                   Mat position,
                   vector<vc_state> &states,
                   int drone_id)
{

   cout << "[INFO] Bearing control has started" << endl;

   Mat suma1 = Mat::zeros(3, 1, CV_64F), suma2 = Mat::zeros(3, 1, CV_64F);
   Mat temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);

   for (int32_t i = 0; i < states[drone_id - 1].params.seguimiento.rows; i++)
   {
      int opc = states[drone_id - 1].params.control;
      if (opc == 0)
      {
         // Control with position
         cout << "[INFO] Control with global position" << endl;
         // temp = desired_bearings.col(i);
         temp = states[drone_id - 1].params.bearing.col(i);
         temp2.at<double>(0, 0) = states[drone_id - 1].Vx - states[(int)states[drone_id - 1].params.seguimiento.at<double>(i, 0) - 1].Vx;
         temp2.at<double>(1, 0) = states[drone_id - 1].Vy - states[(int)states[drone_id - 1].params.seguimiento.at<double>(i, 0) - 1].Vy;
         temp2.at<double>(2, 0) = states[drone_id - 1].Vz - states[(int)states[drone_id - 1].params.seguimiento.at<double>(i, 0) - 1].Vz;
         suma1 -= projOrtog(temp) * (states[drone_id - 1].Kv * position.col(i) + states[drone_id - 1].Kw * temp2);
      }
      else if (opc == 1)
      {
         // Control with difference of bearing
         cout << "[INFO] Control with difference of bearing" << endl;
         suma1 += (-actual_bearing.col(i) - states[drone_id - 1].params.bearing.col(i));
      }
      else if (opc == 2)
      {
         // Control with bearing ortogonal projection
         cout << "[INFO] Control with bearing ortogonal projection" << endl;
         temp = actual_bearing.col(i);
         suma2 -= projOrtog(temp) * (states[drone_id - 1].params.bearing.col(i));
      }
      else if (opc == 3)
      {
         // Control with difference of bearings and orthogonal projection
         cout << "[INFO] Control with difference of bearings and orthogonal projection" << endl;
         temp = actual_bearing.col(i);
         suma1 += -actual_bearing.col(i) - states[drone_id - 1].params.bearing.col(i);
         suma2 -= projOrtog(temp) * (states[drone_id - 1].params.bearing.col(i));
      }
   }

   // Error calculation
   states[drone_id - 1].error = norm(suma1) + norm(suma2);
   states[drone_id - 1].error_pix = states[drone_id - 1].error;

   double l0_Kp = states[drone_id - 1].Kv_max, linf_Kp = states[drone_id - 1].Kv;
   double lambda_Kp = (l0_Kp - linf_Kp) * exp(-(75 * states[drone_id - 1].error) / (l0_Kp - linf_Kp)) + linf_Kp;

   double l0_Kv = states[drone_id - 1].Kw_max, linf_Kv = states[drone_id - 1].Kw;
   double lambda_Kv = (l0_Kv - linf_Kv) * exp(-(-0.005 * states[drone_id - 1].error) / (l0_Kv - linf_Kv)) + linf_Kv;

   cout << endl
        << "[INFO] Lambda kp: " << linf_Kp << " < " << lambda_Kp << " < " << l0_Kp << endl
        << "[INFO] Lambda kv: " << l0_Kv << " < " << lambda_Kv << " < " << linf_Kv << endl;

   states[drone_id - 1].lambda_kp = lambda_Kp;
   states[drone_id - 1].lambda_kv = lambda_Kv;

   // Update the velocity
   Mat suma3 = suma1 + suma2;

   Mat tempSign = signMat(suma3);
   states[drone_id - 1].integral_error += states[drone_id - 1].dt * tempSign;
   Mat tempError = robust(suma3);

   Mat U = lambda_Kp * tempError - lambda_Kv * states[drone_id - 1].integral_error;
   states[drone_id - 1].Vx = U.at<double>(0, 0);
   states[drone_id - 1].Vy = U.at<double>(1, 0);
   states[drone_id - 1].Vz = U.at<double>(2, 0);

   cout << "Desired bearing: " << states[drone_id - 1].params.bearing << endl;
   cout << "Actual bearing: " << actual_bearing << endl;

   return 0;
}

Mat projOrtog(Mat &x)
{
   Mat Px = (Mat::eye(3, 3, CV_64F) - x * x.t());
   return Px;
}