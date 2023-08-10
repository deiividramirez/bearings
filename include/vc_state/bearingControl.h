#include "vc_state/vc_state.h"

class bearingControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;

   // double LastYaw;
   int droneID;

   vector<vc_state> drones;

   double t0L = 0.0, tfL = 1.0;

   bearingControl()
   {
   }

   bearingControl(vc_state *stated, int droneID)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;
      this->droneID = droneID;

      cout << GREEN_C << "\n[INFO] Getting desired data for bearing control for drone " << droneID << RESET_C << endl;

      if (this->getDesiredData() < 0)
      {
         cout << RED_C << "[ERROR] Desired ArUco not found" << RESET_C << endl;
         ros::shutdown();
         exit(-1);
      }
      cout << GREEN_C << "[INFO] Desired data obtained" << RESET_C << endl;
   }

   bearingControl(vc_state *stated, vector<vc_state> drones, int droneID)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;
      this->droneID = droneID;

      this->drones = drones;

      cout << GREEN_C << "\n[INFO] Getting desired data for bearing control" << RESET_C << endl;

      if (this->getDesiredData() < 0)
      {
         cout << RED_C << "[ERROR] Desired ArUco not found" << RESET_C << endl;
         ros::shutdown();
         exit(-1);
      }
      cout << GREEN_C << "[INFO] Desired data obtained" << RESET_C << endl;
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
         cout << RED_C << "Exception: " << e.what() << RESET_C << endl;
         return -1;
      }

      cout << GREEN_C << "\n[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }
      cout << RESET_C << endl;

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
               cout << GREEN_C << "[INFO] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " detected" << RESET_C << endl;
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

            temporal2.row(i) = (Kinv * temporal.row(i).t()).t();
         }
         temporal2.colRange(0, 2).convertTo((*this->state).desired.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
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

         (*this->state).desired.bearings.at<double>(0, marker_index) = temporal.at<double>(2, 0);
         (*this->state).desired.bearings.at<double>(1, marker_index) = -temporal.at<double>(0, 0);
         (*this->state).desired.bearings.at<double>(2, marker_index) = -temporal.at<double>(1, 0);
      }

      (*this->state).desired.markerIds = markerIds;
      (*this->state).desired.markerCorners = markerCorners;

      (*this->state).desired.img = this->imgDesired;
      (*this->state).desired.imgGray = this->imgDesiredGray;

      return 0;
   }

   int getActualData(Mat imgActual)
   {
      vector<int> markerIds;
      vector<vector<Point2f>> markerCorners, rejectedCandidates;
      Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

      try
      {
         aruco::detectMarkers(imgActual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      }
      catch (Exception &e)
      {
         cout << RED_C << "Exception: " << e.what() << RESET_C << endl;
         return -1;
      }

      cout << GREEN_C << "\n[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }
      cout << RESET_C << endl;

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
               // cout << "[INFO] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " detected" << endl;
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
         Mat Kinv;

         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);
         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[indice][i].x;
            temporal.at<float>(i, 1) = markerCorners[indice][i].y;
            temporal.at<float>(i, 2) = 1;

            temporal2.row(i) = (Kinv * temporal.row(i).t()).t();
         }
         temporal2.colRange(0, 2).convertTo((*this->state).actual.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.colRange(0, 2).convertTo((*this->state).actual.points.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
         temporal.convertTo(temporal, CV_64F);

         Mat printi = (*this->state).actual.normPoints - (*this->state).desired.normPoints;

         Mat p1 = temporal.row(0);
         Mat p2 = temporal.row(1);
         Mat p3 = temporal.row(2);
         Mat p4 = temporal.row(3);

         Mat pMedio = puntoMedio(p1, p2, p3, p4);

         Mat temp2 = (*this->state).params.Kinv * pMedio;
         temporal = temp2 / norm(temp2);

         (*this->state).actual.bearings.at<double>(0, marker_index) = temporal.at<double>(2, 0);
         (*this->state).actual.bearings.at<double>(1, marker_index) = -temporal.at<double>(0, 0);
         (*this->state).actual.bearings.at<double>(2, marker_index) = -temporal.at<double>(1, 0);
      }

      (*this->state).actual.markerIds = markerIds;
      (*this->state).actual.markerCorners = markerCorners;

      (*this->state).actual.img = imgActual;
      cvtColor(imgActual, (*this->state).actual.imgGray, COLOR_BGR2GRAY);

      return 0;
   }

   int getVels(Mat imgActual)
   {
      if (this->getActualData(imgActual) < 0)
      {
         cout << RED_C << "[ERROR] Actual ArUco not found" << RESET_C << endl;
         return -1;
      }

      cout << GREEN_C << "[INFO] Getting velocities from Bearing-Only" << RESET_C << endl;

      Mat suma1 = Mat::zeros(3, 1, CV_64F), suma2 = Mat::zeros(3, 1, CV_64F);
      Mat suma1_w = Mat::zeros(3, 3, CV_64F);

      Mat temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);

      int opc = (*this->state).params.control;

      if (opc == 4)
      {
         if (getHomography() < 0)
         {
            cout << RED_C << "[ERROR] Homography not found" << RESET_C << endl;
            return -1;
         }
      }

      for (int32_t i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         temp = (*this->state).actual.bearings.col(i);
         if (opc == 0)
         {
            cout << GREEN_C << "[INFO] Control with global position it is not available at the moment" << RESET_C << endl;
            exit(-1);
         }
         else if (opc == 1)
         {
            cout << GREEN_C << "[INFO] Control with difference of bearing" << RESET_C << endl;
            suma1 += ((*this->state).actual.bearings.col(i) - (*this->state).desired.bearings.col(i));
         }
         else if (opc == 2)
         {
            cout << GREEN_C << "[INFO] Control with bearing ortogonal projection" << RESET_C << endl;
            suma2 -= projOrtog(temp) * ((*this->state).desired.bearings.col(i));
         }
         else if (opc == 3)
         {
            cout << GREEN_C << "[INFO] Control with difference of bearings and orthogonal projection" << RESET_C << endl;
            suma1 += (*this->state).actual.bearings.col(i) - (*this->state).desired.bearings.col(i);
            suma2 -= projOrtog(temp) * ((*this->state).desired.bearings.col(i));
         }
         else if (opc == 4)
         {
            cout << GREEN_C << "[INFO] Control with with bearing ortogonal projection and homography" << RESET_C << endl;
            suma1 -= projOrtog(temp) * ((*this->state).I3 + (*this->state).Qi[i]) / 2.0 * (*this->state).desired.bearings.col(i);
            suma1_w -= ((*this->state).Qi[i].t() - (*this->state).Qi[i]);
         }
         else if (opc == 5)
         {
            cout << GREEN_C << "[INFO] Control with with bearing ortogonal projection - GROUND TRUTH" << RESET_C << endl;
            Mat QiQj = composeR(
                           (*this->state).groundTruth.at<double>(3, 0),
                           (*this->state).groundTruth.at<double>(4, 0),
                           (*this->state).groundTruth.at<double>(5, 0))
                           .t() *
                       composeR(
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(3, 0),
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(4, 0),
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(5, 0));

            suma1 -= projOrtog(temp) * ((*this->state).I3 + QiQj) / 2.0 * (*this->state).desired.bearings.col(i);
            suma1_w -= (QiQj.t() - QiQj);
         }
         else if (opc == 6)
         {
            cout << GREEN_C << "[INFO] Control with with difference of bearings and homography" << RESET_C << endl;
            suma1 += temp - ((*this->state).I3 + (*this->state).Qi[i]) / 2.0 * (*this->state).desired.bearings.col(i);
            suma1_w -= ((*this->state).Qi[i].t() - (*this->state).Qi[i]);
         }
         else if (opc == 7)
         {
            cout << GREEN_C << "[INFO] Control with with difference of bearings - GROUND TRUTH" << RESET_C << endl;
            Mat QiQj = composeR(
                           (*this->state).groundTruth.at<double>(3, 0),
                           (*this->state).groundTruth.at<double>(4, 0),
                           (*this->state).groundTruth.at<double>(5, 0))
                           .t() *
                       composeR(
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(3, 0),
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(4, 0),
                           (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(5, 0));

            suma1 += temp - ((*this->state).I3 + QiQj) / 2.0 * (*this->state).desired.bearings.col(i);
            suma1_w -= (QiQj.t() - QiQj);
         }
      }

      // Update the velocity
      Mat suma3 = suma1 + suma2;

      // Error calculation
      // Mat Error = (*this->state).actual.bearings - (*this->state).desired.bearings;
      Mat Error = abs(suma3);

      (*this->state).error = norm(Error, NORM_L2);
      (*this->state).error_pix = norm((*this->state).actual.normPoints - (*this->state).desired.normPoints, NORM_L2);

      double error_x = Error.at<double>(0, 0);
      double error_y = Error.at<double>(1, 0);
      double error_z = Error.at<double>(2, 0);

      cout << endl
           << "[INFO] Error in x: " << error_x << " y: " << error_y << " z: " << error_z << endl;

      double smooth = 1;
      if ((*this->state).t <= tfL)
         smooth = (1 - cos(M_PI * ((*this->state).t - t0L) / (tfL - t0L))) * .5;

      if (opc == 4 || opc == 5 || opc == 6 || opc == 7)
      {
         double l0_Kv = (*this->state).Kw_max, linf_Kv = (*this->state).Kw;
         double kw1 = smooth * ((l0_Kv - linf_Kv) * exp(-((*this->state).kw_prima * 2 * error_x) / (l0_Kv - linf_Kv)) + linf_Kv);
         double kw2 = smooth * ((l0_Kv - linf_Kv) * exp(-((*this->state).kw_prima * 2 * error_y) / (l0_Kv - linf_Kv)) + linf_Kv);
         double kw3 = smooth * ((l0_Kv - linf_Kv) * exp(-((*this->state).kw_prima * 2 * error_z) / (l0_Kv - linf_Kv)) + linf_Kv);

         (*this->state).lambda_kw = (kw1 + kw2 + kw3) / 3.0;
         (*this->state).Vyaw = (*this->state).lambda_kw * suma1_w.at<double>(1, 0);
      }

      double l0_Kp = (*this->state).Kv_max, linf_Kp = (*this->state).Kv;
      double kp1 = smooth * ((l0_Kp - linf_Kp) * exp(-((*this->state).kv_prima * 1.75 * error_x) / (l0_Kp - linf_Kp)) + linf_Kp);
      double kp2 = smooth * ((l0_Kp - linf_Kp) * exp(-((*this->state).kv_prima * 2 * error_y) / (l0_Kp - linf_Kp)) + linf_Kp);
      double kp3 = smooth * ((l0_Kp - linf_Kp) * exp(-((*this->state).kv_prima * 2 * error_z) / (l0_Kp - linf_Kp)) + linf_Kp);
      (*this->state).lambda_kvp = (kp1 + kp2 + kp3) / 3.0;

      double l0_Kv_i = (*this->state).Kv_i_max, linf_Kv_i = (*this->state).Kv_i;
      double kv1 = smooth * ((l0_Kv_i - linf_Kv_i) * exp(-((*this->state).kv_i_prima * 2 * error_x) / (l0_Kv_i - linf_Kv_i)) + linf_Kv_i);
      double kv2 = smooth * ((l0_Kv_i - linf_Kv_i) * exp(-((*this->state).kv_i_prima * 2 * error_y) / (l0_Kv_i - linf_Kv_i)) + linf_Kv_i);
      double kv3 = smooth * ((l0_Kv_i - linf_Kv_i) * exp(-((*this->state).kv_i_prima * 2 * error_z) / (l0_Kv_i - linf_Kv_i)) + linf_Kv_i);
      (*this->state).lambda_kvi = (kv1 + kv2 + kv3) / 3.0;

      cout << YELLOW_C << endl
           << "[INFO]\nkvp_x: " << kp1 << " kvp_y: " << kp2 << " kvp_z: " << kp3 << endl
           << "kv_i_x: " << kv1 << " kv_i_y: " << kv2 << " kv_i_z: " << kv3 << endl
           << "Lambda kp: " << l0_Kp << " < " << (*this->state).lambda_kvp << " < " << linf_Kp << endl
           << "Lambda kv_i: " << linf_Kv_i << " < " << (*this->state).lambda_kvi << " < " << l0_Kv_i << endl
           << RESET_C
           << endl;

      Mat tempSign = signMat(suma3);
      (*this->state).integral_error += (*this->state).dt * tempSign;
      Mat tempError = robust(suma3);

      (*this->state).integral_error_save.at<double>(0, 0) = -kv1 * (*this->state).integral_error.at<double>(0, 0);
      (*this->state).integral_error_save.at<double>(1, 0) = -kv2 * (*this->state).integral_error.at<double>(1, 0);
      (*this->state).integral_error_save.at<double>(2, 0) = -kv3 * (*this->state).integral_error.at<double>(2, 0);

      double Vx = 3 * (kp1 * tempError.at<double>(0, 0) + (*this->state).integral_error_save.at<double>(0, 0));
      double Vy = (kp2 * tempError.at<double>(1, 0) + (*this->state).integral_error_save.at<double>(1, 0));
      double Vz = (kp3 * tempError.at<double>(2, 0) + (*this->state).integral_error_save.at<double>(2, 0));

      clip(Vx);
      clip(Vy);
      clip(Vz);

      (*this->state).Vx = (float)Vx;
      (*this->state).Vy = (float)Vy;
      (*this->state).Vz = (float)Vz;

      cout << "[INFO] Desired bearing: " << (*this->state).desired.bearings << endl;
      cout << "[INFO] Actual bearing: " << (*this->state).actual.bearings << endl;

      return 0;
   }

   int getHomography()
   {
      vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
      Mat actual32 = Mat::zeros(8, 2, CV_32F);
      Mat desired32 = Mat::zeros(8, 2, CV_32F);
      double Hnorm;

      vector<int> indexID;

      for (int32_t i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         // vector<Mat> descomps;

         (*this->state).actual.points.rowRange(i * 4, i * 4 + 4).convertTo(actual32.rowRange(0, 4), CV_32F);
         (*this->state).desired.points.rowRange(i * 4, i * 4 + 4).convertTo(desired32.rowRange(0, 4), CV_32F);
         // (*this->state).actual.points.rowRange(i * 4, i * 4 + 4).convertTo(actual32, CV_32F);
         // (*this->state).desired.points.rowRange(i * 4, i * 4 + 4).convertTo(desired32, CV_32F);

         puntoMedio(actual32.row(0), actual32.row(1)).copyTo(actual32.row(4));
         puntoMedio(actual32.row(1), actual32.row(2)).copyTo(actual32.row(5));
         puntoMedio(actual32.row(2), actual32.row(3)).copyTo(actual32.row(6));
         puntoMedio(actual32.row(3), actual32.row(0)).copyTo(actual32.row(7));

         puntoMedio(desired32.row(0), desired32.row(1)).copyTo(desired32.row(4));
         puntoMedio(desired32.row(1), desired32.row(2)).copyTo(desired32.row(5));
         puntoMedio(desired32.row(2), desired32.row(3)).copyTo(desired32.row(6));
         puntoMedio(desired32.row(3), desired32.row(0)).copyTo(desired32.row(7));

         Mat mascara;
         // findHomography(actual32, desired32, 0).convertTo((*this->state).Hi[i], CV_64F);
         // findHomography(actual32, desired32, mascara, RANSAC, 3).convertTo((*this->state).Hi[i], CV_64F);
         findHomography(actual32, desired32, LMEDS).convertTo((*this->state).Hi[i], CV_64F);
         // findHomography(actual32, desired32, RHO, 3).convertTo((*this->state).Hi[i], CV_64F);

         if ((*this->state).Hi[i].empty())
         {
            cout << RED_C << "[ERROR] Homography " << i << " not found" << RESET_C << endl;
            return -1;
         }

         Hnorm = sqrt((*this->state).Hi[i].at<double>(0, 0) * (*this->state).Hi[i].at<double>(0, 0) +
                      (*this->state).Hi[i].at<double>(1, 0) * (*this->state).Hi[i].at<double>(1, 0) +
                      (*this->state).Hi[i].at<double>(2, 0) * (*this->state).Hi[i].at<double>(2, 0));
         (*this->state).Hi[i] /= Hnorm;

         Mat QiQj = composeR(
                        (*this->state).groundTruth.at<double>(3, 0),
                        (*this->state).groundTruth.at<double>(4, 0),
                        (*this->state).groundTruth.at<double>(5, 0))
                        .t() *
                    composeR(
                        (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(3, 0),
                        (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(4, 0),
                        (this->drones[(*this->state).params.seguimiento.at<double>(i, 0) - 1]).groundTruth.at<double>(5, 0));

         vector<Mat> Rs, ts, normals;
         int sols = decomposeHomographyMat((*this->state).Hi[i], (*this->state).params.K, Rs, ts, normals);

         if (sols < 1)
         {
            cout << RED_C << "[ERROR] Decomposition of homography " << i << " not found" << RESET_C << endl;
            return -1;
         }
         else if (sols == 1)
         {
            (*state).Qi[i] = Rs[0];
         }
         else
         {

            cout << GREEN_C << "[INFO] Found " << sols << " solutions for rotation in the homography " << i << endl;

            Mat R1 = Rs[0];
            Mat R2 = Rs[2];

            if (norm(QiQj - R1) < norm(QiQj - R2))
            {
               // cout << "\nR1\n"
               //      << R1 << endl;
               // cout << "|R1 - QiQj|: " << norm(QiQj - R1) << endl;
               // cout << "|R2 - QiQj|: " << norm(QiQj - R2) << endl;
               (*state).Qi[i] = R1;
            }
            else
            {
               // cout << "\nR2\n"
               //      << R2 << endl;
               // cout << "|R2 - QiQj|: " << norm(QiQj - R2) << endl;
               // cout << "|R1 - QiQj|: " << norm(QiQj - R1) << endl;
               (*state).Qi[i] = R2;
            }
         }

         // (*state).Qi[i] = HtoR((*this->state).Hi[i], QiQj);

         // Mat img_matches, imgActual, imgDesired;
         // imgActual = (*this->state).actual.img.clone();
         // imgDesired = (*this->state).desired.img.clone();

         // for (int j = 0; j < actual32.rows; j++)
         // {
         //    circle(imgActual, Point(actual32.at<float>(j, 0), actual32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
         //    circle(imgDesired, Point(desired32.at<float>(j, 0), desired32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
         // }

         // warpPerspective(imgActual, img_matches, (*this->state).Hi[i], imgDesired.size());
         // hconcat(imgDesired, img_matches, img_matches);

         // string name = "Homography " + to_string(i);
         // namedWindow(name, WINDOW_NORMAL);
         // resizeWindow(name, 850, 240);
         // imshow(name, img_matches);
         // waitKey(1);
      }

      return 0;
   }

   Mat HtoR(Mat Homography, Mat QiQj)
   {
      Mat U, S, Vt;
      SVD::compute(Homography, S, U, Vt);

      double s1 = S.at<double>(0) / S.at<double>(1);
      double s3 = S.at<double>(2) / S.at<double>(1);

      double zeta = s1 - s3;

      Mat ab = Mat::zeros(2, 1, CV_64F);
      ab.at<double>(0) = sqrt(1 - s3 * s3);
      ab.at<double>(1) = sqrt(s1 * s1 - 1);
      normalize(ab, ab);

      Mat cd = Mat::zeros(2, 1, CV_64F);
      cd.at<double>(0) = 1 + s1 * s3;
      cd.at<double>(1) = sqrt(1 - s3 * s3) * sqrt(s1 * s1 - 1);
      normalize(cd, cd);

      Mat ef = Mat::zeros(2, 1, CV_64F);
      ef.at<double>(0) = -ab.at<double>(1) / s1;
      ef.at<double>(1) = -ab.at<double>(0) / s3;
      normalize(ef, ef);

      Mat v1, v3;
      Vt.row(0).copyTo(v1);
      Vt.row(2).copyTo(v3);

      Mat n1 = ab.at<double>(1) * v1 - ab.at<double>(0) * v3;
      Mat n2 = ab.at<double>(1) * v1 + ab.at<double>(0) * v3;

      Mat temp1 = Mat::zeros(3, 3, CV_64F);
      temp1.at<double>(0, 2) = cd.at<double>(1);
      temp1.at<double>(0, 0) = cd.at<double>(0);
      temp1.at<double>(1, 1) = 1;
      temp1.at<double>(2, 0) = -cd.at<double>(1);
      temp1.at<double>(2, 2) = cd.at<double>(0);

      Mat temp2 = Mat::zeros(3, 3, CV_64F);
      temp2.at<double>(0, 0) = cd.at<double>(0);
      temp2.at<double>(0, 2) = -cd.at<double>(1);
      temp2.at<double>(1, 1) = 1;
      temp2.at<double>(2, 0) = cd.at<double>(1);
      temp2.at<double>(2, 2) = cd.at<double>(0);

      Mat R1 = (U * temp1 * Vt).t();
      Mat R2 = (U * temp2 * Vt).t();

      // cout << "R1: " << R1 << endl;
      // cout << "QiQj: " << QiQj << endl;
      // cout << "R2: " << R2 << endl;

      if (norm(QiQj - R1) < norm(QiQj - R2))
      {
         // cout << "\nR1\n"
         //      << R1 << endl;
         // cout << "|R1 - QiQj|: " << norm(QiQj - R1) << endl;
         // cout << "|R2 - QiQj|: " << norm(QiQj - R2) << endl;
         return R1;
      }
      else
      {
         // cout << "\nR2\n"
         //      << R2 << endl;
         // cout << "|R2 - QiQj|: " << norm(QiQj - R2) << endl;
         // cout << "|R1 - QiQj|: " << norm(QiQj - R1) << endl;
         return R2;
      }
   }

   Mat cameraPoseFromHomography(Mat H, Mat toPose)
   {
      Mat pose = Mat::eye(3, 3, CV_64F); // 3x4 matrix, the camera pose
      double norm1 = (double)norm(H.col(0));
      double norm2 = (double)norm(H.col(1));
      double tnorm = (norm1 + norm2) / 2.0; // Normalization value

      // cout << "H:\n " << H << endl;

      Mat p1 = H.col(0);    // Pointer to first column of H
      Mat p2 = pose.col(0); // Pointer to first column of pose (empty)

      cv::normalize(p1, p2); // Normalize the rotation, and copies the column to pose
      // cout << "p1:\n " << p1 << endl;
      // cout << "p2:\n " << p2 << endl;

      p1 = H.col(1);    // Pointer to second column of H
      p2 = pose.col(1); // Pointer to second column of pose (empty)

      cv::normalize(p1, p2); // Normalize the rotation and copies the column to pose
      // cout << "p1:\n " << p1 << endl;
      // cout << "p2:\n " << p2 << endl;

      p1 = pose.col(0);
      p2 = pose.col(1);

      Mat p3 = p1.cross(p2); // Computes the cross-product of p1 and p2
      Mat c2 = pose.col(2);  // Pointer to third column of pose
      p3.copyTo(c2);         // Third column is the crossproduct of columns one and two
      // cout << "p3:\n " << p3 << endl;
      // cout << "c2:\n " << c2 << endl;

      // pose.col(3) = H.col(2) / tnorm; // vector t [R|t] is the last column of pose
      // cout << "H.col(2):\n " << H.col(2) << endl;
      // cout << "tnorm:\n " << tnorm << endl;

      pose.copyTo(toPose);

      // cout << "Pose antes:\n" << pose << endl;
      Mat descom = decomposeR(pose);
      // cout << "Pose despues:\n" << descom << endl;
      return descom;
   }
};