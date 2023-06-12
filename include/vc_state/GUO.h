#include "vc_state/vc_state.h"
#include <opencv2/core/cuda.hpp>

class GUO
{
public:
   Mat imgDesired;
   Mat imgDesiredGray;
   vc_state *state;
   int mode;

   // ORB detector;
   vector<KeyPoint> keypoints1, keypoints2;
   Mat descriptors1, descriptors2;
   vector<vector<DMatch>> matches;
   vector<Point2f> actualPoints, desiredPoints;

   Ptr<ORB> detector;
   FlannBasedMatcher matcher;
   vector<DMatch> good_matches;

   bool firstTime = false;

   Mat oldImage;
   Mat oldControl;

   GUO()
   {
   }

   GUO(vc_state *stated, int mode)
   {
      this->mode = mode;
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->state = stated;

      cout << "\n[INFO] Getting desired data for GUO control...";

      if (this->getDesiredData() < 0)
      {
         cout << "[ERROR] Desired ArUco not found" << endl;
         ros::shutdown();
         exit(-1);
      }

      // Setting the ORB detector
      this->detector = ORB::create((*stated).params.nfeatures,
                                   (*stated).params.scaleFactor,
                                   (*stated).params.nlevels,
                                   (*stated).params.edgeThreshold,
                                   (*stated).params.firstLevel,
                                   (*stated).params.WTA_K,
                                   (*stated).params.scoreType,
                                   (*stated).params.patchSize,
                                   (*stated).params.fastThreshold);

      matcher = FlannBasedMatcher(new flann::LshIndexParams(20, 10, 2));

      cout << "[INFO] Desired data obtained" << endl;
   }

   int getDesiredData()
   {
      (*this->state).desired.img = this->imgDesired;
      cvtColor((*this->state).desired.img, (*this->state).desired.imgGray, COLOR_BGR2GRAY);
      this->imgDesiredGray = (*this->state).desired.imgGray;

      if (this->mode == 0)
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

         (*this->state).desired.points = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);
         (*this->state).desired.normPoints = Mat::zeros(4 * (*this->state).params.seguimiento.rows, 2, CV_64F);

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
               cout << "[ERROR] Marker " << (*this->state).params.seguimiento.at<double>(marker_index, 0) << " not detected" << endl;
               return -1;
            }

            Mat temporal = Mat::zeros(4, 3, CV_32F);
            Mat temporal2 = Mat::zeros(4, 3, CV_32F), temporal3;

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
         }

         (*this->state).desired.markerIds = markerIds;
         (*this->state).desired.markerCorners = markerCorners;
         this->toSphere((*this->state).desired.points, &(*this->state).desired.inSphere);
      }
      else if (this->mode == 1)
      {
         this->detector->detectAndCompute(this->imgDesiredGray, noArray(), this->keypoints1, this->descriptors1);
         cout << "[INFO] Keypoints detected: " << this->keypoints1.size() << endl;
         // drawKeypoints(this->imgDesired, this->keypoints1, this->imgDesired, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
         // namedWindow("Desired", WINDOW_NORMAL);
         // resizeWindow("Desired", 960, 540);
         // imshow("Desired", this->imgDesired);
         // waitKey(0);
      }
      return 0;
   }

   int getActualData(Mat actualImg)
   {
      (*this->state).actual.img.copyTo(this->oldImage);
      (*this->state).actual.img = actualImg;
      cvtColor((*this->state).actual.img, (*this->state).actual.imgGray, COLOR_BGR2GRAY);
      this->good_matches.clear();

      if (this->mode == 0)
      {
         vector<int> markerIds;
         vector<vector<Point2f>> markerCorners, rejectedCandidates;
         Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
         Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

         try
         {
            aruco::detectMarkers((*this->state).actual.img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
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
            Mat temporal2 = Mat::zeros(4, 3, CV_32F);

            Mat Kinv;
            (*this->state).params.Kinv.convertTo(Kinv, CV_32F);

            for (int i = 0; i < 4; i++)
            {
               temporal.at<float>(i, 0) = markerCorners[marker_index][i].x;
               temporal.at<float>(i, 1) = markerCorners[marker_index][i].y;
               temporal.at<float>(i, 2) = 1;

               temporal2.row(i) = (Kinv * temporal.row(i).t()).t();
            }
            temporal2.colRange(0, 2).convertTo((*this->state).actual.normPoints, CV_64F);
            temporal.colRange(0, 2).convertTo((*this->state).actual.points, CV_64F);

            break;
         }

         (*this->state).actual.markerIds = markerIds;
         (*this->state).actual.markerCorners = markerCorners;
      }
      else if (this->mode == 1)
      {
         Mat Kinv;
         (*this->state).params.Kinv.convertTo(Kinv, CV_32F);

         if (!this->firstTime)
         {
            this->firstTime = true;
            this->detector->detectAndCompute((*this->state).actual.imgGray, noArray(), this->keypoints2, this->descriptors2);
            this->matcher.knnMatch(this->descriptors1, this->descriptors2, this->matches, 2);

            for (int i = 0; i < this->matches.size(); ++i)
            {
               if (this->matches[i][0].distance < this->matches[i][1].distance * (*this->state).params.flann_ratio)
                  this->good_matches.push_back(this->matches[i][0]);
            }

            vector<Point2f> points1, points2;
            for (size_t i = 0; i < this->good_matches.size(); i++)
            {
               points1.push_back(this->keypoints1[this->good_matches[i].queryIdx].pt);
               points2.push_back(this->keypoints2[this->good_matches[i].trainIdx].pt);
            }
            Mat H = findHomography(points1, points2, RANSAC);
            vector<DMatch> new_matches;
            for (size_t i = 0; i < this->good_matches.size(); i++)
            {
               Mat p1 = Mat::ones(3, 1, CV_64F);
               p1.at<double>(0, 0) = this->keypoints1[this->good_matches[i].queryIdx].pt.x;
               p1.at<double>(1, 0) = this->keypoints1[this->good_matches[i].queryIdx].pt.y;
               p1 = H * p1;
               p1 = p1 / p1.at<double>(2, 0);
               double dist = sqrt(pow(p1.at<double>(0, 0) - this->keypoints2[this->good_matches[i].trainIdx].pt.x, 2) + pow(p1.at<double>(1, 0) - this->keypoints2[this->good_matches[i].trainIdx].pt.y, 2));
               if (dist < 5)
               {
                  new_matches.push_back(this->good_matches[i]);
               }
            }
            this->good_matches = new_matches;

            if (this->good_matches.size() <= 5)
            {
               cout << "[ERROR] There are no good matches" << endl;
               return -1;
            }

            cout << "[INFO] Good matches: " << this->good_matches.size() << endl;

            Orden();

            Mat temporal3 = Mat::zeros(4, 3, CV_32F);
            Mat temporal4 = Mat::zeros(4, 3, CV_32F);

            for (int i = 0; i < 4; i++)
            {
               temporal3.at<float>(i, 0) = (*this->state).desired.points.at<double>(i, 0);
               temporal3.at<float>(i, 1) = (*this->state).desired.points.at<double>(i, 1);
               temporal3.at<float>(i, 2) = 1;

               temporal4.row(i) = (Kinv * temporal3.row(i).t()).t();
            }
            temporal4.colRange(0, 2).convertTo((*this->state).desired.normPoints, CV_64F);
            this->toSphere((*this->state).desired.points, &(*this->state).desired.inSphere);

            // Mat img_matches;
            // drawMatches(this->imgDesired, this->keypoints1, actualImg, this->keypoints2, this->good_matches, img_matches, Scalar::all(-1),
            //             Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // namedWindow("Good Matches", WINDOW_NORMAL);
            // resizeWindow("Good Matches", 960, 270);
            // imshow("Good Matches", img_matches);
            // waitKey(0);

            // ros::shutdown();
            // exit(-1);
         }
         else
         {
            Mat oldGray;
            cvtColor(this->oldImage, oldGray, COLOR_BGR2GRAY);

            Mat new_points, status, error;
            Mat actualTemp;
            (*this->state).actual.points.convertTo(actualTemp, CV_32F);
            calcOpticalFlowPyrLK(oldGray, (*this->state).actual.imgGray, actualTemp, new_points, status, error);

            for (int i = 0; i < new_points.rows; i++)
            {
               circle((*this->state).actual.img, (*this->state).actual.points.at<Point2d>(i, 0), 3, Scalar(0, 0, 255), -1);
               circle((*this->state).actual.img, new_points.at<Point2f>(i, 0), 3, Scalar(0, 255, 0), -1);

               circle(this->oldImage, (*this->state).actual.points.at<Point2d>(i, 0), 3, Scalar(0, 0, 255), -1);
               circle(this->oldImage, new_points.at<Point2f>(i, 0), 3, Scalar(0, 255, 0), -1);
            }

            // cout << "Old: " << (*this->state).actual.points << endl;
            // cout << "New: " << new_points << endl;
            new_points.convertTo((*this->state).actual.points, CV_64F);

            // cout << "status: " << status << endl;
            // cout << "error: " << error << endl;

            // namedWindow("Old Matches", WINDOW_NORMAL);
            // resizeWindow("Old Matches", 960, 540);
            // imshow("Old Matches", this->oldImage);
            // waitKey(1);

            // namedWindow("Good Matches", WINDOW_NORMAL);
            // resizeWindow("Good Matches", 960, 540);
            // imshow("Good Matches", (*this->state).actual.img);
            // waitKey(0);
         }

         Mat temporal = Mat::zeros(4, 3, CV_32F);
         Mat temporal2 = Mat::zeros(4, 3, CV_32F);

         for (int i = 0; i < 4; i++)
         {
            temporal.at<float>(i, 0) = (*this->state).actual.points.at<double>(i, 0);
            temporal.at<float>(i, 1) = (*this->state).actual.points.at<double>(i, 1);
            temporal.at<float>(i, 2) = 1;

            temporal2.row(i) = (Kinv * temporal.row(i).t()).t();
         }
         temporal2.colRange(0, 2).convertTo((*this->state).actual.normPoints, CV_64F);
         
         // ros::shutdown();
         // exit(-1);
      }

      this->toSphere((*this->state).actual.points, &(*this->state).actual.inSphere);

      return 0;
   }

   void changeMode(int mode)
   {
      cout << "\n[INFO] Changing mode to " << mode << endl;
      this->mode = mode;
      this->firstTime = false;
      this->imgDesired = (*this->state).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);

      this->oldControl = Mat::zeros(1, 4, CV_64F);
      this->oldControl.at<double>(0, 0) = (*this->state).Vx;
      this->oldControl.at<double>(0, 1) = (*this->state).Vy;
      this->oldControl.at<double>(0, 2) = (*this->state).Vz;
      this->oldControl.at<double>(0, 3) = (*this->state).Vyaw;

      if (this->getDesiredData() < 0)
      {
         cout << "[ERROR] Desired points in picture not found" << endl;
         ros::shutdown();
         exit(-1);
      }
   }

   int getVels(Mat img // Image to be processed
   )
   {
      cout << "\n[INFO] Getting velocities from GUO control..." << endl;

      Mat U, U_temp, L, Lo;
      vector<vecDist> distancias;
      this->getActualData(img);

      cout << "Actual points: " << (*this->state).actual.points << endl;
      cout << "Actual sphere: " << (*this->state).actual.inSphere << endl;
      cout << "Desired points: " << (*this->state).desired.points << endl;
      cout << "Desired sphere: " << (*this->state).desired.inSphere << endl;

      // Send images points to sphere model by generic camera model
      // toSphere((*this->state).actual.points, (*this->state).actual.inSphere);

      // Calculate the distances between the points in the sphere
      distances((*this->state).desired.inSphere, (*this->state).actual.inSphere, distancias, (*this->state).params);

      // Get interaction matrix and error vector with distances
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

   int MinBy(Mat puntos, Mat desired, Mat *ordenFinal)
   {
      // Make buuble sort with norm of the difference between points and key
      *ordenFinal = Mat::zeros(1, 4, CV_32S) - 1;
      Mat orden = Mat::zeros(1, puntos.rows, CV_32S);
      Mat p1, p2;
      vector<Mat> keys;

      keys.push_back((Mat_<double>(1, 2) << 480, 270));
      keys.push_back((Mat_<double>(1, 2) << 480, 810));
      keys.push_back((Mat_<double>(1, 2) << 1440, 270));
      keys.push_back((Mat_<double>(1, 2) << 1440, 810));

      int conteoExterno = 0;
      for (Mat key : keys)
      {
         p1 = desired.clone();
         p2 = puntos.clone();
         for (int i = 0; i < p2.rows; i++)
         {
            orden.at<int>(0, i) = i;
         }

         for (int i = 0; i < p2.rows; i++)
         {
            for (int j = 0; j < p2.rows - 1; j++)
            {
               Mat diff1 = p2.row(j) - key;
               Mat diff2 = p2.row(i) - key;
               if (norm(diff1) > norm(diff2))
               {
                  double temp = p2.at<double>(j, 0);
                  double temp3 = p1.at<double>(j, 0);
                  p1.at<double>(j, 0) = p1.at<double>(i, 0);
                  p2.at<double>(j, 0) = p2.at<double>(i, 0);
                  p1.at<double>(i, 0) = temp3;
                  p2.at<double>(i, 0) = temp;

                  temp = p2.at<double>(j, 1);
                  temp3 = p1.at<double>(j, 1);
                  p1.at<double>(j, 1) = p1.at<double>(i, 1);
                  p2.at<double>(j, 1) = p2.at<double>(i, 1);
                  p1.at<double>(i, 1) = temp3;
                  p2.at<double>(i, 1) = temp;

                  int temp2 = orden.at<int>(0, j);
                  orden.at<int>(0, j) = orden.at<int>(0, i);
                  orden.at<int>(0, i) = temp2;
               }
            }
         }

         int conteo = 0;
         for (int i = 0; i < orden.rows; i++)
         {
            for (int index = 0; index <= (*ordenFinal).cols; index++)
            {
               if ((300. <= p1.at<double>(conteo, 0) <= (1920. - 300.) &&
                    300. <= p1.at<double>(conteo, 1) <= (1080. - 300.)) ||
                   orden.at<int>(0, index) == (*ordenFinal).at<int>(0, index))
               {
                  conteo++;
                  continue;
               }
            }
            (*ordenFinal).at<int>(0, conteoExterno) = orden.at<int>(0, conteo);
            break;
         }

         conteoExterno++;
      }
      return 0;
   }

   int Orden()
   {
      Mat puntosAct, puntosDes;
      for (int i = 0; i < this->good_matches.size(); i++)
      {
         Mat temp = (Mat_<double>(1, 2) << this->keypoints2[this->good_matches[i].trainIdx].pt.x, this->keypoints2[this->good_matches[i].trainIdx].pt.y);
         Mat temp2 = (Mat_<double>(1, 2) << this->keypoints1[this->good_matches[i].queryIdx].pt.x, this->keypoints1[this->good_matches[i].queryIdx].pt.y);
         if (i == 0)
         {
            puntosAct = temp.clone();
            puntosDes = temp2.clone();
         }
         else
         {
            vconcat(puntosAct, temp, puntosAct);
            vconcat(puntosDes, temp2, puntosDes);
         }
      }

      Mat orden;
      MinBy(puntosAct, puntosDes, &orden);

      (this->state)->actual.points = Mat::zeros(4, 2, CV_64F);
      (this->state)->desired.points = Mat::zeros(4, 2, CV_64F);
      (this->state)->actual.points.at<Point2d>(0, 0) = puntosAct.at<Point2d>(orden.at<int>(0, 0), 0);
      (this->state)->actual.points.at<Point2d>(0, 1) = puntosAct.at<Point2d>(orden.at<int>(0, 1), 0);
      (this->state)->actual.points.at<Point2d>(0, 2) = puntosAct.at<Point2d>(orden.at<int>(0, 2), 0);
      (this->state)->actual.points.at<Point2d>(0, 3) = puntosAct.at<Point2d>(orden.at<int>(0, 3), 0);

      (this->state)->desired.points.at<Point2d>(0, 0) = puntosDes.at<Point2d>(orden.at<int>(0, 0), 0);
      (this->state)->desired.points.at<Point2d>(0, 1) = puntosDes.at<Point2d>(orden.at<int>(0, 1), 0);
      (this->state)->desired.points.at<Point2d>(0, 2) = puntosDes.at<Point2d>(orden.at<int>(0, 2), 0);
      (this->state)->desired.points.at<Point2d>(0, 3) = puntosDes.at<Point2d>(orden.at<int>(0, 3), 0);

      // for (int i = 0; i < 4; i++)
      // {
      //    circle(this->state->actual.img, (this->state)->actual.points.at<Point2d>(0, i), 5, Scalar(0, 0, 255), -1);
      //    circle(this->imgDesired, (this->state)->desired.points.at<Point2d>(0, i), 5, Scalar(0, 0, 255), -1);
      // }

      // namedWindow("Actual", WINDOW_NORMAL);
      // resizeWindow("Actual", 960, 540);
      // imshow("Actual", this->state->actual.img);
      // namedWindow("Desired", WINDOW_NORMAL);
      // resizeWindow("Desired", 960, 540);
      // imshow("Desired", this->imgDesired);
      // waitKey(0);

      return 0;
   }
};