#include "vc_state/vc_state.h"

class bearingControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;
   double LastYaw;

   vector<vc_state> drones;

   bearingControl()
   {
   }

   bearingControl(vc_state *stated)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;
      this->LastYaw = (*this->state).Yaw;

      cout << "\n[INFO] Getting desired data for bearing control" << endl;

      if (this->getDesiredData() < 0)
      {
         cout << "[ERROR] Desired ArUco not found" << endl;
         ros::shutdown();
         exit(-1);
      }
      cout << "[INFO] Desired data obtained" << endl;
   }

   bearingControl(vc_state *stated, vector<vc_state> drones)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;
      this->LastYaw = (*this->state).Yaw;

      this->drones = drones;

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

            // temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            // temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }
         cout << "temporal2 brefore " << temporal2 << endl;

         // temporal3.convertTo((*this->state).desired.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
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

         double b1 = temporal.at<double>(2, 0);
         double b2 = -temporal.at<double>(0, 0);

         // (*this->state).desired.bearings.at<double>(0, marker_index) = cos((*state).Yaw) * b1 - sin((*state).Yaw) * b2;
         // (*this->state).desired.bearings.at<double>(1, marker_index) = sin((*state).Yaw) * b1 + cos((*state).Yaw) * b2;
         (*this->state).desired.bearings.at<double>(0, marker_index) = b1;
         (*this->state).desired.bearings.at<double>(1, marker_index) = b2;
         (*this->state).desired.bearings.at<double>(2, marker_index) = -temporal.at<double>(1, 0);
      }

      // cout << "Desired bearing: " << (*this->state).desired.bearings << endl;
      // cout << "State desired: " << (*this->state).params.bearing << endl;
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

            // temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            // temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }

         // temporal3.convertTo((*this->state).actual.normPoints.rowRange(marker_index * 4, marker_index * 4 + 4), CV_64F);
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
      // if (this->getActualData(imgActual) < 0 || getHomography() < 0)
      if (this->getActualData(imgActual) < 0)
      {
         cout << "[ERROR] Actual ArUco not found" << endl;
         return -1;
      }

      cout << "[INFO] Getting velocities from Bearing-Only" << endl;

      Mat suma1 = Mat::zeros(3, 1, CV_64F), suma2 = Mat::zeros(3, 1, CV_64F);
      Mat suma1_w = Mat::zeros(3, 3, CV_64F);

      Mat temp = Mat::zeros(3, 1, CV_64F), temp2 = Mat::zeros(3, 1, CV_64F);

      int opc = (*this->state).params.control;
      for (int32_t i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         if (opc == 0)
         {
            // Control with position
            cout << "[INFO] Control with global position it is not available at the moment" << endl;
            exit(-1);
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
         else if (opc == 4)
         {
            temp = (*this->state).actual.bearings.col(i);
            // suma1 -= projOrtog(temp) * ((*this->state).I3 + (*this->state).Qi[i]) / 2.0 * (*this->state).desired.bearings.col(i);
            // suma1_w -= ((*this->state).Qi[i].t() - (*this->state).Qi[i]);

            Mat QiQj = composeR(
                           (*this->state).groundTruth.at<double>(3, 0),
                           (*this->state).groundTruth.at<double>(4, 0),
                           (*this->state).groundTruth.at<double>(5, 0)
            ).t() * composeR( 
                                 (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(3, 0),
                                 (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(4, 0),
                                 (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(5, 0)
            );

            // cout << "Qi: " << (*this->state).groundTruth.at<double>(3, 0) << " " <<
            //                (*this->state).groundTruth.at<double>(4, 0) << " " <<
            //                (*this->state).groundTruth.at<double>(5, 0) << endl;
            // cout << "Qj: " << (*this->state).params.seguimiento.at<double>(i, 0)-1 << 
            //                " -> " << (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(3, 0) << " " <<
            //                (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(4, 0) << " " <<
            //                (this->drones[(*this->state).params.seguimiento.at<double>(i, 0)-1]).groundTruth.at<double>(5, 0) << endl;
            // cout << "Qi^t * Qj: " << QiQj << endl;

            suma1 -= projOrtog(temp) * ( (*this->state).I3 + QiQj ) / 2.0 * (*this->state).desired.bearings.col(i);
            suma1_w -= (QiQj.t() - QiQj);
         }
      }

      // Update the velocity
      Mat suma3 = suma1 + suma2;

      if (opc == 4)
      {
         (*this->state).Q = (*this->state).Q + (*this->state).dt * (*this->state).Q * (.5 * suma1_w);
         cout << "[INFO] Actual Q: " << (*this->state).Q << endl;
         Mat decom = decomposeR((*this->state).Q);
         cout << "decom" << decom << endl;
         (*this->state).Vyaw = decom.at<double>(2, 0);
         // suma3 = (*this->state).R * (*this->state).Q * suma3;
         // suma3 = (*this->state).Q * suma3;
         // (*this->state).Vyaw = -(*this->state).Yaw;
         // (*this->state).Yaw = decom.at<double>(2, 0);
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

      Mat tempSign = signMat(suma3);
      (*this->state).integral_error += (*this->state).dt * tempSign;
      Mat tempError = robust(suma3);

      Mat U_trans = lambda_Kp * tempError - lambda_Kv * (*this->state).integral_error;
      (*this->state).Vx = (float)U_trans.at<double>(0, 0);
      (*this->state).Vy = (float)U_trans.at<double>(1, 0);
      (*this->state).Vz = (float)U_trans.at<double>(2, 0);

      // cout << "Desired bearing: " << (*this->state).desired.bearings << endl;
      // cout << "Actual bearing: " << (*this->state).actual.bearings << endl;

      return 0;
   }

   int getHomography()
   {
      vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
      Mat actual32 = Mat::zeros(8, 2, CV_32F);
      Mat desired32 = Mat::zeros(8, 2, CV_32F);
      double Hnorm;

      vector<int> indexID;

      // for (int32_t i = 0; i < 1; i++)
      for (int32_t i = 0; i < (*this->state).actual.points.rows / 4; i++)
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

         // findHomography(actual32, desired32, 0).convertTo((*this->state).Hi[i], CV_64F);
         // findHomography(actual32, desired32, RANSAC, 3).convertTo((*this->state).Hi[i], CV_64F);
         findHomography(actual32, desired32, LMEDS).convertTo((*this->state).Hi[i], CV_64F);
         // findHomography(actual32, desired32, RHO, 3).convertTo((*this->state).Hi[i], CV_64F);

         if ((*this->state).Hi[i].empty())
         {
            cout << "[ERROR] Homography " << i << " not found" << endl;
            return -1;
         }

         cout << "Homography " << i << ": " << (*this->state).Hi[i] << endl;

         // Normalization to ensure that ||c1|| = 1
         Hnorm = sqrt((*this->state).Hi[i].at<double>(0, 0) * (*this->state).Hi[i].at<double>(0, 0) +
                      (*this->state).Hi[i].at<double>(1, 0) * (*this->state).Hi[i].at<double>(1, 0) +
                      (*this->state).Hi[i].at<double>(2, 0) * (*this->state).Hi[i].at<double>(2, 0));
         (*this->state).Hi[i] /= Hnorm;

         (*state).Qi[i] = HtoR((*this->state).Hi[i]);

         // Mat descom = decomposeR((*state).Qi[i]);
         // cout << "R" << i << ": " << (*state).Qi[i] << endl;
         // cout << "angles" << i << ": " << descom << endl;

         // (*this->state).Roll = descom.at<double>(0, 0);
         // (*this->state).Pitch = descom.at<double>(1, 0);
         // (*this->state).Vyaw = descom.at<double>(2, 0);

         // cout << "H" << i << ": " << (*this->state).Hi[i] << endl;

         // int sols = decomposeHomographyMat((*this->state).Hi[i], (*state).params.K, Rs_decomp, ts_decomp, normals_decomp);
         // for (int i = 0; i < sols; i++)
         // {
         //    cout << "R" << i << " = " << Rs_decomp[i] << endl;
         //    cout << "t" << i << " = " << ts_decomp[i] << endl;
         //    cout << "n" << i << " = " << normals_decomp[i] << endl << endl;
         //    // cout << "R-tn" << i << " = " << Rs_decomp[i] - ts_decomp[i] * normals_decomp[i].t() << endl
         //    //      << endl;
         // }

         // Mat actual = cameraPoseFromHomography((*this->state).Hi[i], (*state).Qi[i]);
         // cout << "Pose: \n"
         //      << (*state).Qi[i] << endl;

         // for (int i = 0; i < sols; i = i + 2)
         // {
         //    descomps.push_back((*state).R * decomposeR(Rs_decomp[i]));
         // }

         // actual.at<double>(0, 0) = (*this->state).Roll;
         // actual.at<double>(1, 0) = (*this->state).Pitch;
         // actual.at<double>(2, 0) = (*this->state).Yaw;

         // double cond1 = norm(descomps[0] - actual);
         // double cond2 = norm(descomps[1] - actual);

         // if (cond1 < cond2)
         // {
         //    descomps[0].copyTo(actual);
         //    (*state).Qi[i] = composeR(descomps[0]);
         // }
         // else
         // {
         //    descomps[1].copyTo(actual);
         //    (*state).Qi[i] = composeR(descomps[1]);
         // }

         // cout << "GOT  -> Roll: " << actual.at<double>(0, 0) << " Pitch: " << actual.at<double>(1, 0) << " Yaw: " << actual.at<double>(2, 0) << endl;
         // cout << "REAL -> Roll: " << (*this->state).Roll << " Pitch: " << (*this->state).Pitch << " Yaw: " << (*this->state).Yaw << endl;

         // cout << "Homography " << i << ": " << (*this->state).Hi[i] << endl;

         Mat img_matches, imgActual, imgDesired;
         imgActual = (*this->state).actual.img.clone();
         imgDesired = (*this->state).desired.img.clone();

         for (int j = 0; j < actual32.rows; j++)
         {
            circle(imgActual, Point(actual32.at<float>(j, 0), actual32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
            circle(imgDesired, Point(desired32.at<float>(j, 0), desired32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
         }

         warpPerspective(imgActual, img_matches, (*this->state).Hi[i], imgDesired.size());
         hconcat(imgDesired, img_matches, img_matches);

         string name = "Homography " + to_string(i);
         namedWindow(name, WINDOW_NORMAL);
         resizeWindow(name, 850, 240);
         imshow(name, img_matches);
         waitKey(1);
      }

      return 0;
   }

   Mat HtoR(Mat Homography)
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

      Mat R1 = U * temp1 * Vt;
      Mat R2 = U * temp2 * Vt;

      Mat decom1 = decomposeR(R1);
      Mat decom2 = decomposeR(R2);

      this->LastYaw = (*this->state).Yaw;

      if (abs(this->LastYaw - decom1.at<double>(2, 0)) < abs(this->LastYaw - decom2.at<double>(2, 0)))
      {
         cout << "decom1: " << decom1 << endl;
         cout << "decom2: " << decom2 << endl;
         return R1;
      }
      else
      {
         cout << "decom2: " << decom2 << endl;
         cout << "decom1: " << decom1 << endl;
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