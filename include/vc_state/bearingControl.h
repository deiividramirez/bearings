#include "vc_state/vc_state.h"

class bearingControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;

   bearingControl()
   {
   }

   bearingControl(vc_state *stated)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;

      cout << "\n[INFO] Getting desired data for bearing control" << endl;

      if (this->getDesiredData() < 0)
      {
         cout << "[ERROR] Desired ArUco not found" << endl;
         ros::shutdown();
         exit(-1);
      }
      cout << "[INFO] Desired data obtained" << endl;
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

      (*this->state).desired.bearings = Mat::zeros(3, (*state).params.seguimiento.rows, CV_64F);
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
         Mat temporal2;
         Mat temporal3 = Mat::zeros(4, 2, CV_32F);
         Mat Kinv;

         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);
         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[indice][i].x;
            temporal.at<float>(i, 1) = markerCorners[indice][i].y;
            temporal.at<float>(i, 2) = 1;

            temporal2 = Kinv * temporal.row(i).t();

            temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }

         temporal3.convertTo((*this->state).desired.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.colRange(0, 2).convertTo((*this->state).desired.points.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.convertTo(temporal, CV_64F);

         Mat p1 = temporal.row(0);
         Mat p2 = temporal.row(1);
         Mat p3 = temporal.row(2);
         Mat p4 = temporal.row(3);

         // Mat pMedio = puntoMedio(p1, p2, p3, p4);
         Mat pMedio = puntoMedio(temporal.row(0), temporal.row(1), temporal.row(2), temporal.row(3));

         Mat temp2 = (*this->state).params.Kinv * pMedio;
         temporal = temp2 / norm(temp2);

         double b1 = temporal.at<double>(2, 0);
         double b2 = -temporal.at<double>(0, 0);

         // (*this->state).desired.bearings.at<double>(0, marker_index) = cos((*state).Yaw) * b1 - sin((*state).Yaw) * b2;
         // (*this->state).desired.bearings.at<double>(1, marker_index) = sin((*state).Yaw) * b1 + cos((*state).Yaw) * b2;
         (*this->state).desired.bearings.at<double>(0, marker_index) = b1;
         (*this->state).desired.bearings.at<double>(1, marker_index) = b2;
         (*this->state).desired.bearings.at<double>(2, marker_index) = -temporal.at<double>(1, 0);
      }

      cout << "Desired bearing: " << (*this->state).desired.bearings << endl;
      cout << "State desired: " << (*this->state).params.bearing << endl;
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

   int getActualData(Mat imgActual)
   {
      vector<int> markerIds;
      vector<vector<Point2f>> markerCorners, rejectedCandidates;
      Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

      try
      {
         aruco::detectMarkers(imgActual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
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

      (*this->state).actual.bearings = Mat::zeros(3, (*state).params.seguimiento.rows, CV_64F);
      (*this->state).actual.points = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);
      (*this->state).actual.normPoints = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);

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
         Mat temporal2;
         Mat temporal3 = Mat::zeros(4, 2, CV_32F);
         Mat Kinv;

         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);
         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[indice][i].x;
            temporal.at<float>(i, 1) = markerCorners[indice][i].y;
            temporal.at<float>(i, 2) = 1;

            temporal2 = Kinv * temporal.row(i).t();

            temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }

         temporal3.convertTo((*this->state).actual.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.colRange(0, 2).convertTo((*this->state).actual.points.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.convertTo(temporal, CV_64F);

         Mat p1 = temporal.row(0);
         Mat p2 = temporal.row(1);
         Mat p3 = temporal.row(2);
         Mat p4 = temporal.row(3);

         Mat pMedio = puntoMedio(p1, p2, p3, p4);

         Mat temp2 = (*this->state).params.Kinv * pMedio;
         temporal = temp2 / norm(temp2);

         // double b1 = temporal.at<double>(2, 0);
         // double b2 = -temporal.at<double>(0, 0);
         // (*this->state).actual.bearings.at<double>(0, marker_index) = cos((*state).Yaw) * b1 - sin((*state).Yaw) * b2;
         // (*this->state).actual.bearings.at<double>(1, marker_index) = sin((*state).Yaw) * b1 + cos((*state).Yaw) * b2;
         (*this->state).actual.bearings.at<double>(0, marker_index) = temporal.at<double>(2, 0);
         (*this->state).actual.bearings.at<double>(1, marker_index) = -temporal.at<double>(0, 0);
         (*this->state).actual.bearings.at<double>(2, marker_index) = -temporal.at<double>(1, 0);
      }

      (*this->state).actual.img = imgActual;
      // (*this->state).actual.imgGray = cvtColor(imgActual, (*this->state).actual.imgGray, COLOR_BGR2GRAY);
      (*this->state).actual.markerIds = markerIds;
      (*this->state).actual.markerCorners = markerCorners;

      // // draw detected markers on the image
      // for (int i = 0; i < (*this->state).actual.points.rows; i++)
      // {
      //    circle((*this->state).actual.img, Point((*this->state).actual.points.at<double>(i, 0), (*this->state).actual.points.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
      // }
      // namedWindow("Actual", WINDOW_NORMAL);
      // cv::resizeWindow("Actual", 550, 310);
      // imshow("Actual", (*this->state).actual.img);
      // waitKey(1);

      return 0;
   }

   int getVels(Mat imgActual)
   {
      if (this->getActualData(imgActual) < 0)
      {
         cout << "[ERROR] Actual ArUco not found" << endl;
         return -1;
      }

      Mat suma1 = Mat::zeros(3, 1, CV_64F), suma2 = Mat::zeros(3, 1, CV_64F);
      Mat temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);

      for (int32_t i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         int opc = (*this->state).params.control;
         if (opc == 0)
         {
            // Control with position
            cout << "[INFO] Control with global position it is not available at the moment" << endl;
            exit(-1);
            // temp = desired_bearings.col(i);
            // temp = (*this->state).params.bearing.col(i);
            // temp2.at<double>(0, 0) = (*this->state).Vx - (*states)[(int)(*this->state).params.seguimiento.at<double>(i, 0) - 1].Vx;
            // temp2.at<double>(1, 0) = (*this->state).Vy - (*states)[(int)(*this->state).params.seguimiento.at<double>(i, 0) - 1].Vy;
            // temp2.at<double>(2, 0) = (*this->state).Vz - (*states)[(int)(*this->state).params.seguimiento.at<double>(i, 0) - 1].Vz;
            // suma1 -= projOrtog(temp) * ((*this->state).Kv * position.col(i) + (*this->state).Kw * temp2);
         }
         else if (opc == 1)
         {
            // Control with difference of bearing
            cout << "[INFO] Control with difference of bearing" << endl;
            suma1 += ((*this->state).actual.bearings.col(i) - (*this->state).desired.bearings.col(i));
         }
         else if (opc == 2)
         {
            // Control with bearing ortogonal projection
            cout << "[INFO] Control with bearing ortogonal projection" << endl;
            temp = (*this->state).actual.bearings.col(i);
            suma2 -= projOrtog(temp) * ((*this->state).desired.bearings.col(i));
         }
         else if (opc == 3)
         {
            // Control with difference of bearings and orthogonal projection
            cout << "[INFO] Control with difference of bearings and orthogonal projection" << endl;
            temp = (*this->state).actual.bearings.col(i);
            suma1 += (*this->state).actual.bearings.col(i) - (*this->state).desired.bearings.col(i);
            suma2 -= projOrtog(temp) * ((*this->state).desired.bearings.col(i));
         }
      }

      // Error calculation
      // (*this->state).error = norm(suma1) + norm(suma2);
      (*this->state).error = norm((*this->state).actual.bearings - (*this->state).desired.bearings, NORM_L2);
      (*this->state).error_pix = norm((*this->state).actual.normPoints - (*this->state).desired.normPoints, NORM_L2);

      double l0_Kp = (*this->state).Kv_max, linf_Kp = (*this->state).Kv;
      double lambda_Kp = (l0_Kp - linf_Kp) * exp(-(-3 * (*this->state).error) / (l0_Kp - linf_Kp)) + linf_Kp;
      (*this->state).lambda_kp = lambda_Kp;

      double l0_Kv = (*this->state).Kw_max, linf_Kv = (*this->state).Kw;
      double lambda_Kv = (l0_Kv - linf_Kv) * exp(-(-0.4 * (*this->state).error) / (l0_Kv - linf_Kv)) + linf_Kv;
      (*this->state).lambda_kv = lambda_Kv;

      cout << endl
           << "[INFO] Lambda kp: " << linf_Kp << " < " << lambda_Kp << " < " << l0_Kp << endl
           << "[INFO] Lambda kv: " << l0_Kv << " < " << lambda_Kv << " < " << linf_Kv << endl;

      // Update the velocity
      Mat suma3 = suma1 + suma2;

      Mat tempSign = signMat(suma3);
      (*this->state).integral_error += (*this->state).dt * tempSign;
      Mat tempError = robust(suma3);

      Mat U = lambda_Kp * tempError - lambda_Kv * (*this->state).integral_error;
      (*this->state).Vx = U.at<double>(0, 0);
      (*this->state).Vy = U.at<double>(1, 0);
      (*this->state).Vz = U.at<double>(2, 0);

      cout << "Desired bearing: " << (*this->state).desired.bearings << endl;
      cout << "Actual bearing: " << (*this->state).actual.bearings << endl;

      return 0;
   }

   Mat projOrtog(Mat &x)
   {
      Mat Px = (Mat::eye(3, 3, CV_64F) - x * x.t());
      return Px;
   }

   Mat puntoMedio(Mat p1, Mat p2, Mat p3, Mat p4)
   {
      Mat pMedio = Mat::zeros(3, 1, CV_64F);
      pMedio.at<double>(0, 0) = (p1.at<double>(0, 0) + p2.at<double>(0, 0) + p3.at<double>(0, 0) + p4.at<double>(0, 0)) / 4;
      pMedio.at<double>(0, 1) = (p1.at<double>(0, 1) + p2.at<double>(0, 1) + p3.at<double>(0, 1) + p4.at<double>(0, 1)) / 4;
      pMedio.at<double>(0, 2) = (p1.at<double>(0, 2) + p2.at<double>(0, 2) + p3.at<double>(0, 2) + p4.at<double>(0, 2)) / 4;
      return pMedio;
   }
};