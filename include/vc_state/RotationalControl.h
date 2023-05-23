#include "vc_state/vc_state.h"

class RotationalControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;

   RotationalControl()
   {
   }

   RotationalControl(vc_state *stated)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;

      cout << "\n[INFO] Getting desired data for Rotational control...";

      // if (this->getDesiredData() < 0)
      // {
      //    cout << "[ERROR] Desired ArUco not found" << endl;
      //    ros::shutdown();
      //    exit(-1);
      // }
      // cout << "[INFO] Desired data obtained" << endl;
   }

   int getDesiredData()
   {
      vector<int> markerIds;
      vector<vector<Point2f>> markerCorners, rejectedCandidates;
      Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

      try
      {
         aruco::detectMarkers(this->imgDesired, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      }
      catch (Exception &e)
      {
         cout << "Exception: " << e.what() << endl;
         return -1;
      }

      cout << "\n[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }
      cout << endl;

      int indice;

      // cout << "[INFO] Called getBearing for drone " << drone_id << endl;

      (*this->state).desired.points = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);
      (*this->state).desired.normPoints = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);

      for (int32_t marker_index = 0; marker_index < (*this->state).params.seguimiento.rows; marker_index++)
      {
         indice = -1;
         for (int i = 0; i < markerIds.size(); i++)
         {
            if (markerIds[i] == (int)(*this->state).params.seguimiento.at<double>(marker_index, 0))
            {
               cout << "[INFO] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " detected" << endl;
               // cout << "[INFO] Marker corners: " << markerCorners[i] << endl;
               indice = i;
               break;
            }
         }

         if (indice == -1)
         {
            cout << "[ERROR] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " not detected" << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 3, CV_32F);
         Mat temporal2 = Mat::zeros(4, 3, CV_32F);
         // Mat temporal3 = Mat::zeros(4, 2, CV_32F);
         Mat Kinv;

         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);
         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[indice][i].x;
            temporal.at<float>(i, 1) = markerCorners[indice][i].y;
            temporal.at<float>(i, 2) = 1;

            temporal2.row(i) = Kinv * temporal.row(i).t();

            // temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            // temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }

         
      }


      (*this->state).desired.img = this->imgDesired;
      (*this->state).desired.imgGray = this->imgDesiredGray;
      (*this->state).desired.markerIds = markerIds;
      (*this->state).desired.markerCorners = markerCorners;

      // // draw detected markers on the image
      // for (int i = 0; i < (*this->state).desired.points.rows; i++)
      // {
      //    circle((*this->state).desired.img, Point((*this->state).desired.points.at<double>(i, 0), (*this->state).desired.points.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
      // }
      // namedWindow("Desired", WINDOW_NORMAL);
      // cv::resizeWindow("Desired", 550, 310);
      // imshow("Desired", (*this->state).desired.img);
      // waitKey(0);

      return 0;
   }

   int getVels(Mat img)
   {
      cout << "\n[INFO] Getting velocities from Rotational control..." << endl;

      Mat U, U_temp, L, Lo;
      vector<vecDist> distancias;
      // this->getActualData(img);

      L = Lvl();

      double det = 0.0;
      Lo = Moore_Penrose_PInv(L, det);
      if (det < 1e-8)
      {
         cout << "[ERROR] DET = ZERO --> det = " << det << endl;
         return -1;
      }

      Mat ERROR = (*this->state).desired.normPoints - (*this->state).actual.normPoints;

      (*this->state).error_pix = norm(ERROR, NORM_L2);

      cout << "[INFO] Error actual en pix " << (*this->state).error_pix << endl;

      // Choosing the gain for the control law
      double l0_Kp = 1 * (*this->state).Kv_max, linf_Kp = 10 * (*this->state).Kv;
      // double lambda_Kp = (l0_Kp - linf_Kp) * exp(-(-50 * (*this->state).error_pix) / (l0_Kp - linf_Kp)) + linf_Kp;
      double lambda_Kp = .5;

      cout << endl
           << "[INFO] Lambda kp: " << l0_Kp << " < " << lambda_Kp << " < " << linf_Kp << endl;

      Mat ERROR_I = Mat::zeros(2 * (*this->state).actual.normPoints.rows, 1, CV_64F);
      for (int i = 0; i < (*this->state).actual.normPoints.rows; i++)
      {
         ERROR_I.at<double>(2 * i, 0) = ERROR.at<double>(i, 0);
         ERROR_I.at<double>(2 * i + 1, 0) = ERROR.at<double>(i, 1);
      }

      U_temp = -lambda_Kp * Lo * ERROR_I;
      (*this->state).Vyaw = U_temp.at<double>(1, 0);

      // free memory
      U_temp.release();
      L.release();
      Lo.release();

      return 0;
   }

   Mat Lvl()
   {
      // This is classic image-based visual servoing
      Mat L = Mat::zeros(2 * (*this->state).actual.normPoints.rows, 3, CV_64F);

      for (int i = 0; i < (*this->state).actual.normPoints.rows; i++)
      {
         double u = (*this->state).actual.normPoints.at<double>(i, 0);
         double v = (*this->state).actual.normPoints.at<double>(i, 1);

         // cout << "U: " << u << " V: " << v << endl;

         L.at<double>(2 * i, 0) = u * v;
         L.at<double>(2 * i, 1) = -(1 + u * u);
         L.at<double>(2 * i, 2) = v;

         L.at<double>(2 * i + 1, 0) = 1 + v * v;
         L.at<double>(2 * i + 1, 1) = -u * v;
         L.at<double>(2 * i + 1, 2) = -u;
      }

      return L;
   }
};