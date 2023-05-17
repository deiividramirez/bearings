#include "vc_state/vc_state.h"

class GUO
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   vc_state *state;

   GUO()
   {
   }

   GUO(vc_state *stated)
   {
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;

      cout << "\n[INFO] Getting desired data for GUO control...";

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
      }

      (*this->state).desired.img = this->imgDesired;
      (*this->state).desired.imgGray = this->imgDesiredGray;
      (*this->state).desired.markerIds = markerIds;
      (*this->state).desired.markerCorners = markerCorners;

      this->toSphere((*this->state).desired.points, &(*this->state).desired.inSphere);

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

   int getActualData(Mat actualImg)
   {
      vector<int> markerIds;
      vector<vector<Point2f>> markerCorners, rejectedCandidates;
      Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

      this->imgActual = actualImg;

      try
      {
         aruco::detectMarkers(this->imgActual, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      }
      catch (Exception &e)
      {
         cout << "Exception: " << e.what() << endl;
         return -1;
      }

      cout << "[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }
      cout << endl;

      int marker_index = -1;
      for (int indexesXLM = 0; indexesXLM < (*this->state).params.seguimiento.cols; indexesXLM++)
      {

         for (int i = 0; i < markerIds.size(); i++)
         {
            if (markerIds[i] == (int)((*this->state).params.seguimiento.at<double>(indexesXLM)))
            {
               cout << "[INFO] Marker " << (int)(*this->state).params.seguimiento.at<double>(indexesXLM) << " have been detected." << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] All markers in " << (*this->state).params.seguimiento << " not found" << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 3, CV_32F);
         Mat temporal2;
         Mat temporal3 = Mat::zeros(4, 2, CV_32F);
         Mat Kinv;

         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);

         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = markerCorners[marker_index][i].x;
            temporal.at<float>(i, 1) = markerCorners[marker_index][i].y;
            temporal.at<float>(i, 2) = 1;

            temporal2 = Kinv * temporal.row(i).t();

            temporal3.at<float>(i, 0) = temporal2.at<float>(0, 0) / temporal2.at<float>(2, 0);
            temporal3.at<float>(i, 1) = temporal2.at<float>(1, 0) / temporal2.at<float>(2, 0);
         }

         temporal3.convertTo((*this->state).actual.normPoints, CV_64F);
         temporal.colRange(0, 2).convertTo((*this->state).actual.points, CV_64F);

         break;
      }

      // cout << "normPoints: " << (*this->state).actual.normPoints << endl;
      // cout << "points: " << (*this->state).actual.points << endl;

      (*this->state).actual.img = this->imgDesired;
      (*this->state).actual.imgGray = this->imgDesiredGray;
      (*this->state).actual.markerIds = markerIds;
      (*this->state).actual.markerCorners = markerCorners;

      this->toSphere((*this->state).actual.points, &(*this->state).actual.inSphere);

      // // draw detected markers on the image
      // for (int i = 0; i < (*this->state).actual.points.rows; i++)
      // {
      //    circle((*this->state).actual.img, Point((*this->state).actual.points.at<double>(i, 0), (*this->state).actual.points.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
      // }
      // imshow("Desired", (*this->state).actual.img);
      // waitKey(0);

      return 0;
   }

   int getVels(Mat img // Image to be processed
   )
   {
      cout << "\n[INFO] Getting velocities from GUO control..." << endl;

      Mat U, U_temp, L, Lo;
      vector<vecDist> distancias;
      this->getActualData(img);

      // Send images points to sphere model by generic camera model
      // toSphere((*this->state).actual.points, (*this->state).actual.inSphere);

      // Calculate the distances between the points in the sphere
      distances((*this->state).desired.inSphere, (*this->state).actual.inSphere, distancias, (*this->state).params);
      // and sorting these distance for choose the greater ones
      // sort(distancias.begin(), distancias.end(), mayorQue);

      // // Get interaction matrix and error vector with distances
      L = Lvl((*this->state).actual.inSphere, distancias, (*this->state).params);
      Mat ERROR = Mat::zeros(distancias.size(), 1, CV_64F);

      for (int i = 0; i < distancias.size(); i++)
      {
         ERROR.at<double>(i, 0) = (double)distancias[i].dist2 - (double)distancias[i].dist;
      }

      // Get the Penrose pseudo-inverse of the interaction matrix
      double det = 0.0;
      Lo = Moore_Penrose_PInv(L, det);
      if (det < 1e-8)
      {
         cout << "[ERROR] DET = ZERO --> det = " << det << endl;
         return -2;
      }

      (*this->state).error = norm(ERROR, NORM_L1);
      cout << "[INFO] Error actual: " << (*this->state).error << endl;

      // Mat ERROR_PIX = (*this->state).actual.points - (*this->state).desired.points;
      // (*this->state).error_pix = norm(ERROR_PIX, NORM_L2);
      // cout << "[INFO] Error pix: " << (*this->state).error_pix << endl;

      // Choosing the gain for the control law
      double l0_Kp = (*this->state).Kv_max, linf_Kp = (*this->state).Kv;
      double lambda_Kp = (l0_Kp - linf_Kp) * exp(-(-.3 * (*this->state).error) / (l0_Kp - linf_Kp)) + linf_Kp;

      double l0_Kv = (*this->state).Kw_max, linf_Kv = (*this->state).Kw;
      double lambda_Kv = (l0_Kv - linf_Kv) * exp(-(-0.005 * (*this->state).error) / (l0_Kv - linf_Kv)) + linf_Kv;

      cout << endl
           << "[INFO] Lambda kp: " << l0_Kp << " < " << lambda_Kp << " < " << linf_Kp << endl
           << "[INFO] Lambda kv: " << l0_Kv << " < " << lambda_Kv << " < " << linf_Kv << endl;

      (*this->state).lambda_kp = lambda_Kp;
      (*this->state).lambda_kv = lambda_Kv;

      Mat tempSign = signMat(ERROR);
      (*this->state).integral_error6 += (*this->state).dt * tempSign;

      Mat tempError = robust(ERROR);
      U_temp = Lo * (-lambda_Kp * tempError - lambda_Kv * (*this->state).integral_error6);

      // // cout << "[INFO] Error: " << ERROR.t() << endl;
      // // cout << "[INFO] Error robusto: " << tempError.t() << endl;
      // // exit(-1);

      // FIll with zeros the control law in rotation 3x1
      U = Mat::zeros(6, 1, CV_64F);
      U_temp.copyTo(U.rowRange(0, 3));

      // Send the control law to the camera
      if ((*this->state).params.camara == 1)
      {
         (*this->state).Vx = -(float)U.at<double>(2, 0);
         (*this->state).Vy = (float)U.at<double>(0, 0);
         (*this->state).Vz = (float)U.at<double>(1, 0);
      }
      else
      {
         (*this->state).Vx = (float)U.at<double>(1, 0);
         (*this->state).Vy = (float)U.at<double>(0, 0);
         (*this->state).Vz = (float)U.at<double>(2, 0);
      }

      // Free the memory
      U.release();
      U_temp.release();
      L.release();
      Lo.release();
      ERROR.release();
      distancias.clear();

      return 0;
   }

   void toSphere(Mat points,       // Points in the target image
                 Mat *onSphereSave // Empty matrix for 3D recovery direction on sphere of points
   )
   {
      Mat temp = Mat::zeros(3, 1, CV_64F), tmp;             // Temporal matrix for calculation
      (*onSphereSave) = Mat::zeros(points.rows, 3, CV_64F); // Matrix for 3D recovery direction on sphere of points

      for (int i = 0; i < points.rows; i++)
      {
         // Take the points in target image and add 1 to the last row
         temp.at<double>(0, 0) = points.at<double>(i, 0);
         temp.at<double>(1, 0) = points.at<double>(i, 1);
         temp.at<double>(2, 0) = 1;
         // Invert the matrix of the camera and multiply by the points
         tmp = (*this->state).params.Kinv * temp;
         // // Normalize the points
         (*onSphereSave).row(i) = tmp.t() / norm(tmp);
      }

      // Free the memory
      temp.release();
      tmp.release();
   }

   int distances(Mat p1,                      // Points in the target image
                 Mat p2,                      // Points in the actual image
                 vector<vecDist> &distancias, // Vector of distances struct
                 vc_parameters &params        // Parameters of the camera
   )
   {
      vecDist tmpDist;    // Temporal struct for calculation
      double dist, dist2; // Temporal variables for distance calculation
      int NUM, i, j;      // Variables for the loop

      // NUM = 16; // Number of points to calculate the distance
      NUM = p2.rows; // Number of points to calculate the distance

      cout << "DISTANCIAS" << endl;
      cout << "NUM: " << NUM << endl;

      for (int i = 0; i < NUM; i++)
      {
         // for (int j = 0; j < NUM; j++)
         for (int j = 0; j < i; j++)
         {
            if (i != j)
            {
               double dot1 = (double)(p2.row(i).dot(p2.row(j)));
               double dot2 = (double)(p1.row(i).dot(p1.row(j)));
               dist = sqrt(2 - 2 * dot1);
               dist2 = sqrt(2 - 2 * dot2);
               if (dist <= 1e-9 || dist2 <= 1e-9
                   //     || dot1 > .97
                   //     || dot2 > .97
               )
               {
                  continue;
               }

               tmpDist.i = i;
               tmpDist.j = j;
               if (params.control == 1)
               {
                  tmpDist.dist = 1 / dist;
                  tmpDist.dist2 = 1 / dist2;
               }
               else if (params.control == 2)
               {
                  tmpDist.dist = dist;
                  tmpDist.dist2 = dist2;
               }
               else
               {
                  cout << "[ERROR] Control variable is not valid" << endl;
                  return -1;
               }
               distancias.push_back(tmpDist);
            }
         }
      }
      return 0;
   }

   bool mayorQue(vecDist a, vecDist b)
   {
      return a.dist > b.dist;
   }

   Mat ortoProj(Mat p1)
   {
      Mat I = Mat::eye(3, 3, CV_64F);
      Mat p1Temp = Mat::zeros(3, 1, CV_64F);

      p1Temp.at<double>(0, 0) = p1.at<double>(0, 0);
      p1Temp.at<double>(1, 0) = p1.at<double>(0, 1);
      p1Temp.at<double>(2, 0) = p1.at<double>(0, 2);

      Mat OP = I - p1Temp * p1Temp.t();

      I.release();
      p1Temp.release();
      return OP;
   }

   Mat Lvl(Mat p2s,                    // Points of the actual image in the sphere
           vector<vecDist> &distances, // Vector of distances struct with actual distances
           vc_parameters &params       // Parameters of the camera
   )
   {
      int n = distances.size(); // Number of distances
      // int n = 16; // Number of distances
      // std::cout << std::endl
      //           << "[INFO] Size Interaction Matrix: [" << n << "x3]" << std::endl
      //           << std::endl;

      Mat temp = Mat::zeros(3, 1, CV_64F); // Temp vector for calculation
      Mat L = Mat::zeros(n, 3, CV_64F);    // Interaction matrix
      Mat pi, pj;                          // Temporal points for calculation
      double s;
      cout << (params.control == 1 ? "Control 1: 1/dist" : "Control 2: dist") << endl;
      cout << "n: " << n << endl;
      cout << "distances.size(): " << distances.size() << endl;
      cout << "p2s.size(): " << p2s.size() << endl;
      for (int i = 0; i < n; i++)
      {
         pi = p2s.row(distances[i].i);
         pj = p2s.row(distances[i].j);

         if (params.control == 1)
         {
            s = -distances[i].dist * distances[i].dist * distances[i].dist;
         }
         else if (params.control == 2)
         {
            s = 1 / distances[i].dist;
         }
         else
         {
            cout << "[Error] Control parameter not valid" << endl;
            return L;
         }
         temp = s * ((pi * ortoProj(pj)) + (pj * ortoProj(pi)));
         temp.copyTo(L.row(i));
      }
      temp.release();
      pi.release();
      pj.release();
      return L;
   }
};