#include "vc_state/vc_state.h"

class RotationalControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;

   double t0L = 0.0, tfL = 2.0;

   RotationalControl()
   {
   }

   RotationalControl(vc_state *stated)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;

      cout << "[INFO] Getting desired data for Rotational control..." << endl;

      // if (this->getDesiredData() < 0)
      // {
      //    cout << RED_C << "[ERROR] Desired ArUco not found" << RESET_C << endl;
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
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

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
            cout << RED_C << "[ERROR] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " not detected" << RESET_C << endl;
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
      cout << GREEN_C << "\n[INFO] Getting velocities from Rotational control..." << RESET_C << endl;

      Mat U, U_temp, L, Lo;
      vector<vecDist> distancias;
      // this->getActualData(img);

      L = Lvl();

      double det = 0.0;
      Lo = Moore_Penrose_PInv(L, det);
      if (det < 1e-8)
      {
         cout << RED_C << "[ERROR] DET = ZERO --> det = " << det << RESET_C << endl;
         return -1;
      }

      Mat ERROR = (*this->state).desired.normPoints - (*this->state).actual.normPoints;

      (*this->state).error_pix = norm(ERROR, NORM_L2);
      // cout << "[INFO] Error actual en pix " << (*this->state).error_pix << endl;

      double smooth = 1;
      if ((*this->state).t <= tfL)
         smooth = (1 - cos(M_PI * ((*this->state).t - t0L) / (tfL - t0L))) * .5;

      // Choosing the gain for the control law
      double l0_Kw = (*this->state).Kw_max, linf_Kw = (*this->state).Kw;
      double lambda_Kw = (l0_Kw - linf_Kw) * exp(-((*this->state).kw_prima * (*this->state).error_pix) / (l0_Kw - linf_Kw)) + linf_Kw;
      (*this->state).lambda_kw = smooth * lambda_Kw;

      cout << YELLOW_C << endl
           << "[INFO] Lambda kw: " << l0_Kw << " < " << lambda_Kw << " < " << linf_Kw << RESET_C << endl
           << endl;

      Mat ERROR_I = Mat::zeros(2 * (*this->state).actual.normPoints.rows, 1, CV_64F);
      for (int i = 0; i < (*this->state).actual.normPoints.rows; i++)
      {
         ERROR_I.at<double>(2 * i, 0) = ERROR.at<double>(i, 0);
         ERROR_I.at<double>(2 * i + 1, 0) = ERROR.at<double>(i, 1);
      }

      U_temp = -(*this->state).lambda_kw * Lo * ERROR_I;
      // (*this->state).Vyaw = U_temp.at<double>(1, 0);

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