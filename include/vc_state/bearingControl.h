#include "vc_state/vc_state.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

class bearingControl
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;
   int droneID;

   ofstream timeExcec;
   double extraX = 2.5;

   vector<vc_state> drones;
   vector<Mat> Ris;

   Mat ERROR_MAT;

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

      for (int i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         this->Ris.push_back(Mat::zeros(3, 3, CV_64F));
      }

      timeExcec = ofstream(workspace + "/src/bearings/src/data/out/execution_data_" + to_string(droneID) + ".txt");
      this->ERROR_MAT = Mat::zeros(3, 1, CV_64F);
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

      for (int i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         this->Ris.push_back(Mat::zeros(3, 3, CV_64F));
      }

      timeExcec = ofstream(workspace + "/src/bearings/src/data/out/execution_data_" + to_string(droneID) + ".txt");
      this->ERROR_MAT = Mat::zeros(3, 1, CV_64F);
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

      cout << "Desired bearings: " << (*this->state).desired.bearings << endl;
      cout << "DRONE ID: " << droneID << endl;

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
      double ActualTime = ros::Time::now().toSec();
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

      if (opc == 4 || opc == 6)
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
      this->ERROR_MAT = Error;

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

      double Vx = this->extraX * (kp1 * tempError.at<double>(0, 0) + (*this->state).integral_error_save.at<double>(0, 0));
      double Vy = (kp2 * tempError.at<double>(1, 0) + (*this->state).integral_error_save.at<double>(1, 0));
      double Vz = (kp3 * tempError.at<double>(2, 0) + (*this->state).integral_error_save.at<double>(2, 0));

      clip(Vx);
      clip(Vy);
      clip(Vz);

      (*this->state).Vx = (float)Vx;
      (*this->state).Vy = (float)Vy;
      (*this->state).Vz = (float)Vz;

      // cout << "[INFO] Desired bearing: " << (*this->state).desired.bearings << endl;
      // cout << "[INFO] Actual bearing: " << (*this->state).actual.bearings << endl;

      // (*this->state).Vx = (float)(sin(((*this->state).t) * 3)) / 2;
      // cout << "[INFO] Vx: " << (*this->state).Vx << endl;

      // (*this->state).Vy = (float)(cos(((*this->state).t) * 3)) / 2;
      // cout << "[INFO] Vy: " << (*this->state).Vy << endl;

      // (*this->state).Vz = (float)(cos(((*this->state).t) * 3)) / 2;
      // cout << "[INFO] Vz: " << (*this->state).Vz << endl;

      // (*this->state).Vyaw = (float)(cos(((*this->state).t) * 3)) / 2;
      // cout << "[INFO] Vyaw: " << (*this->state).Vyaw << endl;


      double FinalTime = ros::Time::now().toSec() - ActualTime;
      this->timeExcec << FinalTime << endl;
      cout << GREEN_C << "[INFO] Velocities obtained in " << FinalTime << " seconds" << RESET_C << endl;
      return 0;
   }

   Mat mean(Mat points)
   {
      cout << GREEN_C << "[INFO] Calculating mean" << RESET_C << endl;
      Mat mean = Mat::zeros(1, 2, CV_64F);
      for (int i = 0; i < points.rows; i++)
      {
         mean.at<double>(0, 0) += points.at<double>(i, 0);
         mean.at<double>(0, 1) += points.at<double>(i, 1);
         // cout << "Point " << i << ": " << points.at<float>(i, 0) << " " << points.at<float>(i, 1) << endl;
         // cout << "Mean: " << mean << endl;
      }
      mean /= points.rows;
      // cout << "Final mean " << mean << endl << endl;
      return mean;
   }

   double std(Mat points)
   {
      // get the max of all standant deviation
      double maxstd = 0;
      Mat meanPoints = mean(points);
      double mean1 = meanPoints.at<double>(0, 0);
      double mean2 = meanPoints.at<double>(0, 1);

      double std1 = 0, std2 = 0;
      for (int i = 0; i < points.rows; i++)
      {
         std1 += pow(points.at<double>(i, 0) - mean1, 2);
         std2 += pow(points.at<double>(i, 1) - mean2, 2);
      }
      std1 /= points.rows;
      std2 /= points.rows;
      std1 = sqrt(std1);
      std2 = sqrt(std2);
      if (std1 > std2)
         return std1 + 1e-9;
      else
         return std2 + 1e-9;
   }

   int getHomography()
   {
      Mat actual32 = Mat::zeros(4, 2, CV_64F);
      Mat desired32 = Mat::zeros(4, 2, CV_64F);      

      for (int32_t i = 0; i < (*this->state).params.seguimiento.rows; i++)
      {
         (*this->state).actual.points.rowRange(i * 4, i * 4 + 4).convertTo(actual32.rowRange(0, 4), CV_64F);
         (*this->state).desired.points.rowRange(i * 4, i * 4 + 4).convertTo(desired32.rowRange(0, 4), CV_64F);

         // cout << "Desired " << i << ": " << desired32 << endl;
         // cout << "Actual " << i << ": " << actual32 << endl;

         Mat mascara;
         findtheHomography(desired32, actual32).convertTo((*this->state).Hi[i], CV_64F);

         Mat img_matches, imgActual, imgDesired;
         imgActual = (*this->state).actual.img.clone();
         imgDesired = (*this->state).desired.img.clone();

         for (int j = 0; j < actual32.rows; j++)
         {
            circle(imgActual, Point(actual32.at<float>(j, 0), actual32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
            circle(imgDesired, Point(desired32.at<float>(j, 0), desired32.at<float>(j, 1)), 5, Scalar(0, 0, 255), 2);
         }

         warpPerspective(imgDesired, img_matches, (*this->state).Hi[i], imgDesired.size());
         hconcat(imgActual, img_matches, img_matches);

         string name = "Homography " + to_string(i);
         namedWindow(name, WINDOW_NORMAL);
         resizeWindow(name, 850, 240);
         imshow(name, img_matches);
         waitKey(1);

         (*this->state).Hi[i] = (*this->state).params.Kinv * (*this->state).Hi[i] * (*this->state).params.K;
         (*state).Qi[i] = HtoR((*this->state).Hi[i], i);

         // cout << "Homography " << i << ": " << (*this->state).Hi[i] << endl;
         // cout << "Qi " << i << ": " << (*this->state).Qi[i] << endl;
      }

      return 0;
   }

   Mat findtheHomography(Mat desired, Mat actual)
   {
      cout << GREEN_C << "[INFO] Finding homography" << RESET_C << endl;

      Mat mean1 = mean(desired);
      // cout << "[INFO] Mean1: " << mean1 << endl;
      double maxstd = std(desired);
      // cout << "[INFO] Maxstd: " << maxstd << endl;

      Mat C1 = Mat::zeros(3, 3, CV_64F);
      C1.at<double>(0, 0) = 1 / maxstd;
      C1.at<double>(0, 2) = -mean1.at<double>(0, 0) / maxstd;
      C1.at<double>(1, 1) = 1 / maxstd;
      C1.at<double>(1, 2) = -mean1.at<double>(0, 1) / maxstd;
      C1.at<double>(2, 2) = 1;
      // cout << "[INFO] C1: " << C1 << endl;

      Mat desiredTemp = Mat::ones(desired.rows, 3, CV_64F);
      desired.copyTo(desiredTemp.colRange(0, 2));
      desiredTemp = ((C1 * desiredTemp.t()).t());
      // cout << "[INFO] DesiredTemp: " << desiredTemp << endl;

      Mat mean2 = mean(actual);
      double maxstd2 = std(actual);
      // cout << "[INFO] Mean2: " << mean2 << endl;
      // cout << "[INFO] Maxstd2: " << maxstd2 << endl;

      Mat C2 = Mat::zeros(3, 3, CV_64F);
      C2.at<double>(0, 0) = 1 / maxstd2;
      C2.at<double>(0, 2) = -mean2.at<double>(0, 0) / maxstd2;
      C2.at<double>(1, 1) = 1 / maxstd2;
      C2.at<double>(1, 2) = -mean2.at<double>(0, 1) / maxstd2;
      C2.at<double>(2, 2) = 1;
      // cout << "[INFO] C2: " << C2 << endl;

      Mat actualTemp = Mat::ones(actual.rows, 3, CV_64F);
      actual.copyTo(actualTemp.colRange(0, 2));
      actualTemp = ((C2 * actualTemp.t()).t());
      // cout << "[INFO] ActualTemp: " << actualTemp << endl;

      Mat A = Mat::zeros(2 * actual.rows, 9, CV_64F);
      for (int i = 0; i < actual.rows; i++)
      {
         A.at<double>(2 * i, 0) = desiredTemp.at<double>(i, 0);
         A.at<double>(2 * i, 1) = desiredTemp.at<double>(i, 1);
         A.at<double>(2 * i, 2) = 1;
         // A.at<double>(2*i, 3) = 0;
         // A.at<double>(2*i, 4) = 0;
         // A.at<double>(2*i, 5) = 0;
         A.at<double>(2 * i, 6) = -actualTemp.at<double>(i, 0) * desiredTemp.at<double>(i, 0);
         A.at<double>(2 * i, 7) = -actualTemp.at<double>(i, 0) * desiredTemp.at<double>(i, 1);
         A.at<double>(2 * i, 8) = -actualTemp.at<double>(i, 0);

         // A.at<double>(2*i + 1, 0) = 0;
         // A.at<double>(2*i + 1, 1) = 0;
         // A.at<double>(2*i + 1, 2) = 0;
         A.at<double>(2 * i + 1, 3) = desiredTemp.at<double>(i, 0);
         A.at<double>(2 * i + 1, 4) = desiredTemp.at<double>(i, 1);
         A.at<double>(2 * i + 1, 5) = 1;
         A.at<double>(2 * i + 1, 6) = -actualTemp.at<double>(i, 1) * desiredTemp.at<double>(i, 0);
         A.at<double>(2 * i + 1, 7) = -actualTemp.at<double>(i, 1) * desiredTemp.at<double>(i, 1);
         A.at<double>(2 * i + 1, 8) = -actualTemp.at<double>(i, 1);
      }
      // cout << "[INFO] A: " << A << " - shape: " << A.rows << "x" << A.cols << endl;

      // // convert A to Eigen::MatrixXd
      // Eigen::MatrixXd A_eigen;
      // cv2eigen(A, A_eigen);

      // // compute SVD
      // Eigen::BDCSVD<Eigen::MatrixXd> svd(A_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
      // Eigen::MatrixXd U_eigen = svd.matrixU();
      // Eigen::MatrixXd S_eigen = svd.singularValues();
      // Eigen::MatrixXd V_eigen = svd.matrixV();

      // // convert back to cv::Mat
      // Mat U, S, V;
      // eigen2cv(U_eigen, U);
      // eigen2cv(S_eigen, S);
      // eigen2cv(V_eigen, V);
      
      Mat U, S, V;
      SVD::compute(A, S, U, V, SVD::FULL_UV);

      U = -U;
      V = -V;
      // cout << "[INFO] U: " << U << " - shape: " << U.rows << "x" << U.cols << endl;
      // cout << "[INFO] S: " << S << " - shape: " << S.rows << "x" << S.cols << endl;
      // cout << "[INFO] V: " << V << " - shape: " << V.rows << "x" << V.cols << endl;

      Mat H = V.row(8).reshape(0, 3);
      H = C2.inv() * H * C1;
      H /= H.at<double>(2, 2);
      // cout << "[INFO] H: " << H << endl;
      // exit(0);
      return H;
   }

   Mat HtoR(Mat Homography, int index)
   {
      Mat U, S, V;
      SVD::compute(Homography, S, U, V, SVD::FULL_UV);

      U = -U;
      U.col(2) = -U.col(2);
      V = -V;
      V.row(2) = -V.row(2);

      // cout << "H: " << Homography << endl;
      // cout << "U: " << U << endl;
      // cout << "S: " << S << endl;
      // cout << "V: " << V << endl;

      double s1 = S.at<double>(0) / S.at<double>(1);
      double s3 = S.at<double>(2) / S.at<double>(1);

      double zeta = s1 - s3;

      // cout << "s1: " << s1 << endl;
      // cout << "s3: " << s3 << endl;
      // cout << "zeta: " << zeta << endl;

      double a1 = sqrt(1 - s3 * s3);
      double b1 = sqrt(s1 * s1 - 1);

      Mat ab = Mat::zeros(2, 1, CV_64F);
      ab.at<double>(0) = a1;
      ab.at<double>(1) = b1;
      normalize(ab, ab);

      Mat cd = Mat::zeros(2, 1, CV_64F);
      cd.at<double>(0) = 1 + s1 * s3;
      cd.at<double>(1) = a1 * b1;
      normalize(cd, cd);

      Mat ef = Mat::zeros(2, 1, CV_64F);
      ef.at<double>(0) = -ab.at<double>(1) / s1;
      ef.at<double>(1) = -ab.at<double>(0) / s3;
      normalize(ef, ef);

      // cout << "a1: " << a1 << " -- b1: " << b1 << endl;
      // cout << "ab: " << ab << endl;
      // cout << "cd: " << cd << endl;
      // cout << "ef: " << ef << endl;

      Mat v1, v3;
      V.row(0).convertTo(v1, CV_64F);
      V.row(2).convertTo(v3, CV_64F);

      // cout << "v1: " << v1 << endl;
      // cout << "v3: " << v3 << endl;

      Mat n1 = ab.at<double>(1) * v1 - ab.at<double>(0) * v3;
      Mat n2 = ab.at<double>(1) * v1 + ab.at<double>(0) * v3;

      Mat t1 = zeta * (ef.at<double>(0) * v1 + ef.at<double>(1) * v3);
      Mat t2 = zeta * (ef.at<double>(0) * v1 - ef.at<double>(1) * v3);

      Mat temp1 = Mat::zeros(3, 3, CV_64F);
      temp1.at<double>(0, 0) = cd.at<double>(0);
      temp1.at<double>(0, 2) = cd.at<double>(1);
      temp1.at<double>(1, 1) = 1;
      temp1.at<double>(2, 0) = -cd.at<double>(1);
      temp1.at<double>(2, 2) = cd.at<double>(0);

      Mat temp2 = Mat::zeros(3, 3, CV_64F);
      temp2.at<double>(0, 0) = cd.at<double>(0);
      temp2.at<double>(0, 2) = -cd.at<double>(1);
      temp2.at<double>(1, 1) = 1;
      temp2.at<double>(2, 0) = cd.at<double>(1);
      temp2.at<double>(2, 2) = cd.at<double>(0);

      // cout << "n1: " << n1 << endl;
      // cout << "n2: " << n2 << endl;

      // cout << "n1(2)" << n1.at<double>(0, 2) << endl;
      // cout << "n2(2)" << n2.at<double>(0, 2) << endl;

      if (n1.at<double>(0, 2) < 0)
      {
         n1 = -n1;
         t1 = -t1;
      }
      if (n2.at<double>(0, 2) < 0)
      {
         n2 = -n2;
         t2 = -t2;
      }

      Mat R1 = (U * temp1 * V).t();
      Mat R2 = (U * temp2 * V).t();

      rot2euler(R1, 0);
      cout << "\nR1: " << R1 << endl;
      cout << "----> n1 " << n1 << endl;
      rot2euler(R2, 1);
      cout << "\nR2: " << R2 << endl;
      cout << "----> n2 " << n2 << endl;
      // exit(0);

      if (norm(this->Ris[index]) == 0)
      {
         if (n1.at<double>(0, 2) > n2.at<double>(0, 2))
         {
            cout << "-> R: " << R1 << endl;
            cout << "-> n: " << n1 << endl;
            rot2euler(R1, 2);
            R1.copyTo(this->Ris[index]);
            return R1;
         }
         else
         {
            cout << "-> R: " << R2 << endl;
            cout << "-> n: " << n2 << endl;
            rot2euler(R2, 2);
            R2.copyTo(this->Ris[index]);
            return R2;
         }
      }
      else
      {
         if (norm(this->Ris[index] - R1) < norm(this->Ris[index] - R2))
         {
            rot2euler(R1, 2);
            R1.copyTo(this->Ris[index]);
            return R1;
         }
         else
         {
            rot2euler(R2, 2);
            R2.copyTo(this->Ris[index]);
            return R2;
         }
      }
   }
};