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

   double argMIN1, argMAX1, argMIN2, argMAX2;
   Mat maskMat;

   double t0L = 0.0, tfL = 1.0, lastLKT = 0.0;

   GUO()
   {
   }

   GUO(vc_state *stated, int mode)
   {
      this->mode = mode;
      this->imgDesired = (*stated).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);
      this->state = stated;

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

      // Setting the FLANN matcher
      matcher = FlannBasedMatcher(new flann::LshIndexParams(20, 10, 2));

      argMIN1 = 200;
      argMAX1 = 1920 - 200;

      argMIN2 = 200;
      argMAX2 = 1080 - 200;

      maskMat = Mat::zeros(1080, 1920, CV_8U);
      rectangle(maskMat, Point(argMIN1, argMIN2), Point(argMAX1, argMAX2), Scalar(255, 255, 255), -1);

      cout << GREEN_C << "\n[INFO] Getting desired data for GUO control..." << RESET_C << endl;
      if (this->getDesiredData() < 0)
      {
         cout << RED_C << RED_C << "[ERROR] Desired ArUco not found" << RESET_C << RESET_C << endl;
         ros::shutdown();
         exit(-1);
      }
      cout << GREEN_C << "[INFO] Desired data obtained" << RESET_C << endl;
   }

   int getDesiredData()
   {
      this->imgDesired = (*this->state).desired.img;
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
         (*this->state).desired.points = Mat::zeros((*this->state).params.seguimiento.rows, 2, CV_64F);
         (*this->state).desired.normPoints = Mat::zeros((*this->state).params.seguimiento.rows, 2, CV_64F);

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
            Mat temporal2 = Mat::zeros(4, 3, CV_32F), temporal3;

            Mat Kinv;
            (*this->state).params.Kinv.convertTo(Kinv, CV_32F);

            int indexMark;
            if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 96)
            {
               indexMark = 0;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 97)
            {
               indexMark = 1;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 98)
            {
               indexMark = 3;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 99)
            {
               indexMark = 2;
            }

            temporal.at<float>(marker_index, 0) = markerCorners[indice][indexMark].x;
            temporal.at<float>(marker_index, 1) = markerCorners[indice][indexMark].y;
            temporal.at<float>(marker_index, 2) = 1;

            temporal2.row(marker_index) = (Kinv * temporal.row(marker_index).t()).t();

            temporal2.colRange(0, 2).row(marker_index).convertTo((*this->state).desired.normPoints.row(marker_index), CV_64F);
            temporal.colRange(0, 2).row(marker_index).convertTo((*this->state).desired.points.row(marker_index), CV_64F);
         }

         (*this->state).desired.markerIds = markerIds;
         (*this->state).desired.markerCorners = markerCorners;
         this->toSphere((*this->state).desired.points, &(*this->state).desired.inSphere);
      }
      else if (this->mode == 1)
      {
         this->detector->detectAndCompute(this->imgDesiredGray, maskMat, this->keypoints1, this->descriptors1);
         cout << GREEN_C << "[INFO] Keypoints detected: " << this->keypoints1.size() << RESET_C << endl;

         // Mat temp;
         // drawKeypoints(this->imgDesired, this->keypoints1, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
         // namedWindow("Desired keypoints", WINDOW_NORMAL);
         // resizeWindow("Desired keypoints", 960, 540);
         // imshow("Desired keypoints", temp);
         // waitKey(0);
         // destroyWindow("Desired keypoints");
      }
      return 0;
   }

   int getActualData(Mat actualImg)
   {
      (*this->state).actual.img.copyTo(this->oldImage);
      actualImg.copyTo((*this->state).actual.img);
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
            cout << RED_C << "Exception: " << e.what() << RESET_C << endl;
            return -1;
         }

         cout << GREEN_C << "[INFO] Markers detected: " << markerIds.size() << " with marker ids: ";
         for (int i = 0; i < markerIds.size(); i++)
         {
            cout << markerIds[i] << " ";
         }
         cout << RESET_C << endl;

         int indice;
         (*this->state).actual.points = Mat::zeros((*this->state).params.seguimiento.rows, 2, CV_64F);
         (*this->state).actual.normPoints = Mat::zeros((*this->state).params.seguimiento.rows, 2, CV_64F);

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
            Mat temporal2 = Mat::zeros(4, 3, CV_32F), temporal3;

            Mat Kinv;
            (*this->state).params.Kinv.convertTo(Kinv, CV_32F);

            int indexMark;
            if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 96)
            {
               indexMark = 0;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 97)
            {
               indexMark = 1;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 98)
            {
               indexMark = 3;
            }
            else if ((*this->state).params.seguimiento.at<double>(marker_index, 0) == 99)
            {
               indexMark = 2;
            }

            temporal.at<float>(marker_index, 0) = markerCorners[indice][indexMark].x;
            temporal.at<float>(marker_index, 1) = markerCorners[indice][indexMark].y;
            temporal.at<float>(marker_index, 2) = 1;

            temporal2.row(marker_index) = (Kinv * temporal.row(marker_index).t()).t();
            temporal2.colRange(0, 2).row(marker_index).convertTo((*this->state).actual.normPoints.row(marker_index), CV_64F);

            temporal.colRange(0, 2).row(marker_index).convertTo((*this->state).actual.points.row(marker_index), CV_64F);
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
            cout << GREEN_C << "[INFO] Getting new information from images" << RESET_C << endl;
            this->keypoints2.clear();
            this->descriptors2.release();
            this->matches.clear();
            this->good_matches.clear();

            this->detector->detectAndCompute((*this->state).actual.imgGray, maskMat, this->keypoints2, this->descriptors2);

            // Mat temp;
            // drawKeypoints(actualImg, this->keypoints2, temp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
            // namedWindow("Actual keypoints", WINDOW_NORMAL);
            // resizeWindow("Actual keypoints", 960, 540);
            // imshow("Actual keypoints", temp);
            // waitKey(0);
            // destroyWindow("Actual keypoints");

            this->matcher.knnMatch(this->descriptors1, this->descriptors2, this->matches, 2);

            for (int i = 0; i < this->matches.size(); ++i)
            {
               if (this->matches[i][0].distance < this->matches[i][1].distance * (*this->state).params.flann_ratio)
                  this->good_matches.push_back(this->matches[i][0]);
            }

            cout << "[INFO] Matches: " << this->matches.size() << endl;

            if (this->good_matches.size() <= 5)
            {
               cout << RED_C << "[ERROR] There are no good matches" << RESET_C << endl;
               this->firstTime = false;
               return -1;
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
               cout << RED_C << "[ERROR] There are no good matches" << RESET_C << endl;
               return -1;
            }

            cout << GREEN_C << "[INFO] Good matches: " << this->good_matches.size() << RESET_C << endl;

            if (Orden() < 0)
            {
               cout << RED_C << "[ERROR] Orden function failed" << RESET_C << endl;
               return -1;
            }

            Mat temporal3 = Mat::zeros((*this->state).desired.points.rows, 3, CV_32F);
            Mat temporal4 = Mat::zeros((*this->state).desired.points.rows, 3, CV_32F);

            for (int i = 0; i < (*this->state).desired.points.rows; i++)
            {
               temporal3.at<float>(i, 0) = (*this->state).desired.points.at<double>(i, 0);
               temporal3.at<float>(i, 1) = (*this->state).desired.points.at<double>(i, 1);
               temporal3.at<float>(i, 2) = 1;

               temporal4.row(i) = (Kinv * temporal3.row(i).t()).t();
            }
            temporal4.colRange(0, 2).convertTo((*this->state).desired.normPoints, CV_64F);
            this->toSphere((*this->state).desired.points, &(*this->state).desired.inSphere);

            Mat img_matches;
            drawMatches(this->imgDesired, this->keypoints1, actualImg, this->keypoints2, this->good_matches, img_matches, Scalar::all(-1),
                        Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // namedWindow("Good Matches", WINDOW_NORMAL);
            // resizeWindow("Good Matches", 960, 270);
            // imshow("Good Matches", img_matches);
            // waitKey(0);
            // destroyWindow("Good Matches");

            // ros::shutdown();
            // exit(-1);
            this->firstTime = true;
         }
         else
         {
            Mat oldGray;
            cvtColor(this->oldImage, oldGray, COLOR_BGR2GRAY);

            Mat new_points, status, error;
            Mat actualTemp;
            (*this->state).actual.points.convertTo(actualTemp, CV_32F);
            calcOpticalFlowPyrLK(oldGray, (*this->state).actual.imgGray, actualTemp, new_points, status, error);
            new_points.convertTo((*this->state).actual.points, CV_64F);

            // if (sum(status)[0] < 4)
            // {
            //    cout << RED_C << "[ERROR] There are no good matches" << RESET_C << endl;
            //    this->firstTime = false;
            //    return -1;
            // }

            // for (int i = 0; i < new_points.rows; i++)
            // {
            // circle((*this->state).actual.img, (*this->state).actual.points.at<Point2d>(i, 0), 3, Scalar(0, 0, 255), -1);
            // circle((*this->state).actual.img, new_points.at<Point2f>(i, 0), 3, Scalar(0, 255, 0), -1);

            //    circle(this->oldImage, (*this->state).actual.points.at<Point2d>(i, 0), 3, Scalar(0, 0, 255), -1);
            //    circle(this->oldImage, new_points.at<Point2f>(i, 0), 3, Scalar(0, 255, 0), -1);
            // }

            // namedWindow("Old Matches", WINDOW_NORMAL);
            // resizeWindow("Old Matches", 960, 540);
            // imshow("Old Matches", this->oldImage);
            // waitKey(1);

            // namedWindow("Good Matches", WINDOW_NORMAL);
            // resizeWindow("Good Matches", 960, 540);
            // imshow("Good Matches", (*this->state).actual.img);
            // waitKey(1);
         }

         Mat temporal = Mat::zeros((*this->state).actual.points.rows, 3, CV_32F);
         Mat temporal2 = Mat::zeros((*this->state).actual.points.rows, 3, CV_32F);

         for (int i = 0; i < (*this->state).actual.points.rows; i++)
         {
            temporal.at<float>(i, 0) = (*this->state).actual.points.at<double>(i, 0);
            temporal.at<float>(i, 1) = (*this->state).actual.points.at<double>(i, 1);
            temporal.at<float>(i, 2) = 1;

            temporal2.row(i) = (Kinv * temporal.row(i).t()).t();
         }
         temporal2.colRange(0, 2).convertTo((*this->state).actual.normPoints, CV_64F);
         temporal.colRange(0, 2).convertTo((*this->state).actual.points, CV_64F);
      }

      this->toSphere((*this->state).actual.points, &(*this->state).actual.inSphere);

      return 0;
   }

   void changeMode(int mode)
   {
      cout << MAGENTA_C << "\n[INFO] Changing mode to " << mode << RESET_C << endl;
      this->mode = mode;
      this->firstTime = false;
      this->imgDesired = (*this->state).desired.img;
      cvtColor(imgDesired, this->imgDesiredGray, COLOR_BGR2GRAY);

      t0L = (*this->state).t;
      tfL = (*this->state).t + 1.0;

      this->keypoints1.clear();
      this->descriptors1.release();

      if (this->getDesiredData() < 0)
      {
         cout << RED_C << "[ERROR] Desired points in picture not found" << RESET_C << endl;
         ros::shutdown();
         exit(-1);
      }

      cout << MAGENTA_C << "[INFO] Desired image has been changed" << RESET_C << endl;

      // imshow("Desired", this->imgDesired);
      // waitKey(0);
      // destroyWindow("Desired");
   }

   int getVels(Mat img // Image to be processed
   )
   {
      cout << GREEN_C << "\n[INFO] Getting velocities from GUO control..." << RESET_C << endl;

      Mat U_temp, L, Lo;
      vector<vecDist> distancias;

      if ((*this->state).t - this->lastLKT > 3)
      {
         this->firstTime = false;
         this->lastLKT = (*this->state).t;
      }

      if (this->getActualData(img) < 0)
      {
         cout << RED_C << "[ERROR] Actual points in picture not found" << RESET_C << endl;
         return -1;
      }

      // Calculate the distances between the points in the sphere
      distances((*this->state).desired.inSphere, (*this->state).actual.inSphere, distancias, (*this->state).params);

      // Get interaction matrix and error vector with distances
      L = Lvl((*this->state).actual.inSphere, distancias, (*this->state).params);

      // Get the Penrose pseudo-inverse of the interaction matrix
      double det = 0.0;
      Lo = Moore_Penrose_PInv(L, det);
      if (!(det > 1e-8))
      {
         cout << RED_C << "[ERROR] DET = ZERO --> det = " << det << RESET_C << endl;
         return -1;
      }

      Mat ERROR = Mat::zeros(distancias.size(), 1, CV_64F);
      for (int i = 0; i < distancias.size(); i++)
         ERROR.at<double>(i, 0) = (double)distancias[i].dist2 - (double)distancias[i].dist;

      cout << "[INFO] Error: " << ERROR.t() << endl;
      (*this->state).error = norm(ERROR, NORM_L1);

      // Choosing the gain for the control law
      double smooth = 1;
      if ((*this->state).t < tfL)
         smooth = (1 - cos(M_PI * ((*this->state).t - t0L) / (tfL - t0L))) * .5;

      double l0_Kp = (*this->state).Kv_max, linf_Kp = (*this->state).Kv;
      (*this->state).lambda_kvp = smooth * ((l0_Kp - linf_Kp) * exp(-((*this->state).kv_prima * (*this->state).error) / (l0_Kp - linf_Kp)) + linf_Kp);

      cout << YELLOW_C << endl
           << "[INFO] Lambda kp: " << linf_Kp << " < " << (*this->state).lambda_kvp << " < " << l0_Kp << RESET_C << endl;

      U_temp = -(*this->state).lambda_kvp * Lo * ERROR;
      clip(U_temp);

      (*this->state).Vx = -(float)U_temp.at<double>(2, 0);
      (*this->state).Vy = (float)U_temp.at<double>(0, 0);
      (*this->state).Vz = (float)U_temp.at<double>(1, 0);

      // Free the memory
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
                  cout << RED_C << "[ERROR] Control variable is not valid" << RESET_C << endl;
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
      cout << YELLOW_C << (params.control == 1 ? "Control 1: 1/dist" : "Control 2: dist") << RESET_C << endl;

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
            cout << RED_C << "[Error] Control parameter not valid" << RESET_C << endl;
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
      // Mat orden = Mat::zeros(1, puntos.rows, CV_32S);
      Mat p2;
      vector<Mat> keys;

      // keys.push_back((Mat_<double>(1, 2) << 480, 270));
      // keys.push_back((Mat_<double>(1, 2) << 1440, 270));
      // keys.push_back((Mat_<double>(1, 2) << 1440, 810));
      // keys.push_back((Mat_<double>(1, 2) << 480, 810));
      keys.push_back((Mat_<double>(1, 2) << 0, 0));
      keys.push_back((Mat_<double>(1, 2) << 1920, 0));
      keys.push_back((Mat_<double>(1, 2) << 0, 1080));
      keys.push_back((Mat_<double>(1, 2) << 1920, 1080));

      // Make buuble sort with norm of the difference between points and key
      *ordenFinal = Mat::zeros(1, keys.size(), CV_32S) - 1;

      int conteoExterno = 0;
      for (Mat key : keys)
      {
         vector<int> orden;
         // puntos.copyTo(p2);
         desired.copyTo(p2);

         for (int i = 0; i < p2.rows; i++)
         {
            orden.push_back(i);
         }

         // bubble sort for the points - key
         for (int i = 0; i < p2.rows; i++)
         {
            for (int j = 0; j < p2.rows - i - 1; j++)
            {
               Mat diff1 = key - p2.row(j);
               Mat diff2 = key - p2.row(j + 1);
               if (norm(diff1, NORM_L2) > norm(diff2, NORM_L2))
               {
                  Mat aux;
                  p2.row(j).copyTo(aux);
                  p2.row(j + 1).copyTo(p2.row(j));
                  aux.copyTo(p2.row(j + 1));

                  int auxInt = orden[j];
                  orden[j] = orden[j + 1];
                  orden[j + 1] = auxInt;
               }
            }
         }

         int conteo = 0;
         for (int i = 0; i < orden.size(); i++)
         {
            if (!(argMIN1 <= desired.at<double>(orden[conteo], 0) && desired.at<double>(orden[conteo], 0) <= argMAX1 && argMIN2 <= desired.at<double>(orden[conteo], 1) && desired.at<double>(orden[conteo], 1) <= argMAX2))
            {
               conteo++;
               continue;
            }

            bool flag = false;
            for (int index = 0; index < (*ordenFinal).cols; index++)
            {
               if (orden[conteo] == (*ordenFinal).at<int>(0, index))
               {
                  conteo++;
                  flag = true;
                  break;
               }
               if ((*ordenFinal).at<int>(0, index) != -1)
               {
                  Mat temp = desired.row(orden[conteo]) - desired.row((*ordenFinal).at<int>(0, index));
                  if (norm(temp) < 100)
                  {
                     conteo++;
                     flag = true;
                     break;
                  }
               }
            }

            if (flag)
            {
               continue;
            }
            else
            {
               (*ordenFinal).at<int>(0, conteoExterno) = orden[conteo];
               break;
            }
         }
         // cout << "Key: " << key << " - conteo: " << conteo << " - Orden: " << orden[conteo] << endl;
         conteoExterno++;
      }

      // show the image with all the posible points and the chosen one with the key
      // Mat temporal = this->imgDesired.clone();
      // for (Mat key : keys)
      // {
      //    circle(temporal, key.at<Point2d>(0, 0), 5, Scalar(0, 0, 255), -1);
      // }
      // for (int i = 0; i < 4; i++)
      // {
      //    circle(temporal, desired.at<Point2d>(ordenFinal->at<int>(0, i), 0), 5, Scalar(0, 255, 0), -1);
      // }

      // line(temporal, Point(argMIN1, 0), Point(argMIN1, temporal.rows), Scalar(0, 0, 255), 2);
      // line(temporal, Point(argMAX1, 0), Point(argMAX1, temporal.rows), Scalar(0, 0, 255), 2);
      // line(temporal, Point(0, argMIN2), Point(temporal.cols, argMIN2), Scalar(0, 0, 255), 2);
      // line(temporal, Point(0, argMAX2), Point(temporal.cols, argMAX2), Scalar(0, 0, 255), 2);

      // namedWindow("Desired", WINDOW_NORMAL);
      // resizeWindow("Desired", 960, 540);
      // imshow("Desired", temporal);
      // waitKey(0);
      // ros::shutdown();
      // exit(-1);
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

      for (int i = 0; i < orden.cols; i++)
      {
         if (orden.at<int>(0, i) == -1)
         {
            this->firstTime = false;
            return -1;
         }
      }

      (*this->state).actual.points = Mat::zeros(orden.cols, 2, CV_64F);
      (*this->state).desired.points = Mat::zeros(orden.cols, 2, CV_64F);

      for (int i = 0; i < orden.cols; i++)
      {
         (*this->state).actual.points.at<Point2d>(i, 0) = puntosAct.at<Point2d>(orden.at<int>(0, i), 0);
         (*this->state).desired.points.at<Point2d>(i, 0) = puntosDes.at<Point2d>(orden.at<int>(0, i), 0);
      }

      // (*this->state).actual.points.at<Point2d>(0, 0) = puntosAct.at<Point2d>(orden.at<int>(0, 0), 0);
      // (*this->state).actual.points.at<Point2d>(0, 1) = puntosAct.at<Point2d>(orden.at<int>(0, 1), 0);
      // (*this->state).actual.points.at<Point2d>(0, 2) = puntosAct.at<Point2d>(orden.at<int>(0, 2), 0);
      // (*this->state).actual.points.at<Point2d>(0, 3) = puntosAct.at<Point2d>(orden.at<int>(0, 3), 0);

      // (*this->state).desired.points.at<Point2d>(0, 0) = puntosDes.at<Point2d>(orden.at<int>(0, 0), 0);
      // (*this->state).desired.points.at<Point2d>(0, 1) = puntosDes.at<Point2d>(orden.at<int>(0, 1), 0);
      // (*this->state).desired.points.at<Point2d>(0, 2) = puntosDes.at<Point2d>(orden.at<int>(0, 2), 0);
      // (*this->state).desired.points.at<Point2d>(0, 3) = puntosDes.at<Point2d>(orden.at<int>(0, 3), 0);

      // cout << "[INFO] Actual points: " << (*this->state).actual.points << endl;
      // cout << "[INFO] Desired points: " << (*this->state).desired.points << endl;

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

      // waitKey(0);
      // ros::shutdown();
      // exit(0);

      return 0;
   }
};