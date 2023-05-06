#include "vc_state/vc_state.h"

// using namespace cv;
// using namespace std;

// class GUO;

// // int toSphere(Mat p1, Mat p2, Mat &p1s, Mat &p2s, vc_parameters &params);
// // int distances(Mat p1, Mat p2, vector<vecDist> &distancias, vc_parameters &params);
// // bool mayorQue(vecDist a, vecDist b);
// // Mat ortoProj(Mat p1);
// // Mat Lvl(Mat p2s, vector<vecDist> &distances, vc_parameters &params);

// #include "vc_state/GUO.h"

class GUO
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   Mat imgActual;
   Mat imgActualGray;
   vc_state state;
   // vc_homograpy_matching_result matching_result;

   GUO(vc_state stated)
   {
      this->imgDesired = stated.desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->imgActual = imgActual;
      this->state = stated;

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

      int marker_index = -1;
      for (int i = 0; i < markerIds.size(); i++)
      {

         for (int indexesXLM = 0; indexesXLM < state.params.seguimiento.cols; indexesXLM++)
         {
            if (markerIds[i] == (int)(this->state.params.seguimiento.at<double>(indexesXLM)))
            {
               cout << "[INFO] Marker " << (int)this->state.params.seguimiento.at<double>(indexesXLM) << " have been detected." << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] All markers in " << this->state.params.seguimiento << " not found" << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners[marker_index][0].x, markerCorners[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners[marker_index][1].x, markerCorners[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners[marker_index][2].x, markerCorners[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners[marker_index][3].x, markerCorners[marker_index][3].y);
         temporal.convertTo(this->state.desired.points, CV_64F);

         this->state.desired.img = this->imgDesired;
         this->state.desired.imgGray = this->imgDesiredGray;
         this->state.desired.markerIds = markerIds;
         this->state.desired.markerCorners = markerCorners;

         this->toSphere(this->state.desired.points, this->state.desired.inSphere);

         break;
      }

      // // draw detected markers on the image
      // for (int i = 0; i < this->state.desired.points.rows; i++)
      // {
      //    circle(this->state.desired.img, Point(this->state.desired.points.at<double>(i, 0), this->state.desired.points.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
      // }
      // imshow("Desired", this->state.desired.img);
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

      cout << "\n[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
      for (int i = 0; i < markerIds.size(); i++)
      {
         cout << markerIds[i] << " ";
      }
      cout << endl;

      int marker_index = -1;
      for (int i = 0; i < markerIds.size(); i++)
      {

         for (int indexesXLM = 0; indexesXLM < state.params.seguimiento.cols; indexesXLM++)
         {
            if (markerIds[i] == (int)(this->state.params.seguimiento.at<double>(indexesXLM)))
            {
               cout << "[INFO] Marker " << (int)this->state.params.seguimiento.at<double>(indexesXLM) << " have been detected." << endl;
               marker_index = i;
               break;
            }
         }
         if (marker_index == -1)
         {
            cout << "[ERROR] All markers in " << this->state.params.seguimiento << " not found" << endl;
            return -1;
         }

         Mat temporal = Mat::zeros(4, 2, CV_32F);
         temporal.at<Point2f>(0, 0) = Point2f(markerCorners[marker_index][0].x, markerCorners[marker_index][0].y);
         temporal.at<Point2f>(1, 0) = Point2f(markerCorners[marker_index][1].x, markerCorners[marker_index][1].y);
         temporal.at<Point2f>(2, 0) = Point2f(markerCorners[marker_index][2].x, markerCorners[marker_index][2].y);
         temporal.at<Point2f>(3, 0) = Point2f(markerCorners[marker_index][3].x, markerCorners[marker_index][3].y);
         temporal.convertTo(this->state.actual.points, CV_64F);

         this->state.actual.img = this->imgDesired;
         this->state.actual.imgGray = this->imgDesiredGray;
         this->state.actual.markerIds = markerIds;
         this->state.actual.markerCorners = markerCorners;

         this->toSphere(this->state.actual.points, this->state.actual.inSphere);

         break;
      }

      // // draw detected markers on the image
      // for (int i = 0; i < this->state.actual.points.rows; i++)
      // {
      //    circle(this->state.actual.img, Point(this->state.actual.points.at<double>(i, 0), this->state.actual.points.at<double>(i, 1)), 5, Scalar(0, 0, 255), 2);
      // }
      // imshow("Desired", this->state.actual.img);
      // waitKey(0);

      return 0;
   }

   int getVels(Mat img                                      // Image to be processed
   )
   {

      // // Compute the matching between the images using ORB as detector and descriptor
      // if (compute_descriptors(img, state.params, state.desired_configuration, matching_result) < 0)
      // {
      //         cout << "Error en compute_descriptors" << endl;
      //         return -1;
      // }

      // Temporal matrixes for calculation
      // Mat p1s, p2s, p23D, , 
      Mat U, U_temp, L, Lo;
      // p1s = Mat::zeros(matching_result.p1.rows, 3, CV_64F);
      // p2s = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
      // p23D = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
      vector<vecDist> distancias;
      this->getActualData(img);

      // Send images points to sphere model by generic camera model
      toSphere(this->state.actual.points, this->state.actual.inSphere);

      // Calculate the distances between the points in the sphere
      // and sorting these distance for choose the greater ones
      distances(this->state.desired.inSphere, this->state.actual.inSphere, distancias, state.params);
      // sort(distancias.begin(), distancias.end(), mayorQue);

      // // Get interaction matrix and error vector with distances
      L = Lvl(this->state.actual.inSphere, distancias, state.params);
      Mat ERROR = Mat::zeros(distancias.size(), 1, CV_64F);

      // for (int i = 0; i < 16; i++)
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
         return -1;
      }

      Mat ERROR_PIX = this->state.actual.points - this->state.desired.points;
      this->state.error = norm(ERROR, NORM_L1);
      this->state.error_pix = norm(ERROR_PIX, NORM_L2);

      cout << "[INFO] Error actual: " << this->state.error << endl;
      cout << "[INFO] Error pix: " << this->state.error_pix << endl;

      // Choosing the gain for the control law
      double l0 = state.Kv_max, linf = state.Kv, lprima = .3;
      double lambda_temp = (l0 - linf) * exp(-(lprima * this->state.error) / (l0 - linf)) + linf;
      state.lambda_kp = lambda_temp;

      Mat tempSign = signMat(ERROR);
      state.integral_error6 += state.dt * tempSign;

      Mat tempError = robust(ERROR);
      U_temp = Lo * (-lambda_temp * tempError - 1 / (50 * lambda_temp) * state.integral_error6);

      // // cout << "[INFO] Error: " << ERROR.t() << endl;
      // // cout << "[INFO] Error robusto: " << tempError.t() << endl;
      // // exit(-1);

      // FIll with zeros the control law in rotation 3x1
      U = Mat::zeros(6, 1, CV_64F);
      U_temp.copyTo(U.rowRange(0, 3));
      cout << endl
           << "[INFO] Lambda: " << linf << " < " << lambda_temp << " < " << l0 << endl;
      // cout << "[CONTROL] U = " << U.t() << endl;

      // Send the control law to the camera
      if (state.params.camara == 1)
      {
         state.Vx = -(float)U.at<double>(2, 0);
         state.Vy = (float)U.at<double>(0, 0);
         state.Vz = (float)U.at<double>(1, 0);
      }
      else
      {
         state.Vx = (float)U.at<double>(1, 0);
         state.Vy = (float)U.at<double>(0, 0);
         state.Vz = (float)U.at<double>(2, 0);
      }

      // state.Vroll  = (float) U.at<double>(3,0);
      // state.Vpitch = (float) U.at<double>(4,0);
      state.Vyaw = (float)U.at<double>(5, 0);
      // cout << "Enviadas las velocidades..." << endl;

      U.release();
      U_temp.release();
      L.release();
      Lo.release();
      ERROR.release();
      distancias.clear();
      // p1s.release();
      // p2s.release();
      // p23D.release();

      return 0;
   }

   void toSphere(Mat points,                   // Points in the target image
                Mat &onSphereSave             // Empty matrix for 3D recovery direction on sphere of points 
   )
   {
      Mat temp = Mat::zeros(3, 1, CV_64F), tmp; // Temporal matrix for calculation
      onSphereSave = Mat::zeros(points.rows, 3, CV_64F);     // Matrix for 3D recovery direction on sphere of points 

      for (int i = 0; i < points.rows; i++)
      {
         // Take the points in target image and add 1 to the last row
         temp.at<double>(0, 0) = points.at<double>(i, 0);
         temp.at<double>(1, 0) = points.at<double>(i, 1);
         temp.at<double>(2, 0) = 1;
         // Invert the matrix of the camera and multiply by the points
         tmp = this->state.params.Kinv * temp;
         // Normalize the points
         onSphereSave.row(i) = tmp.t() / norm(tmp);
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
      std::cout << std::endl
                << "[INFO] Size Interaction Matrix: [" << n << "x3]" << std::endl
                << std::endl;

      Mat temp = Mat::zeros(3, 1, CV_64F); // Temp vector for calculation
      Mat L = Mat::zeros(n, 3, CV_64F);    // Interaction matrix
      Mat pi, pj;                          // Temporal points for calculation
      double s;
      cout << (params.control == 1 ? "Control 1: 1/dist" : "Control 2: dist") << endl;
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