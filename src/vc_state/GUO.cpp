#include "vc_state/GUO.h"

int GUO(Mat img,                                      // Image to be processed
        vc_state &state,                              // State of the camera
        vc_homograpy_matching_result &matching_result // Result of the matcher matching
)
{

        // // Compute the matching between the images using ORB as detector and descriptor
        // if (compute_descriptors(img, state.params, state.desired_configuration, matching_result) < 0)
        // {
        //         cout << "Error en compute_descriptors" << endl;
        //         return -1;
        // }

        // Temporal matrixes for calculation
        Mat p1s, p2s, p23D, Lo, U, U_temp, L;
        p1s = Mat::zeros(matching_result.p1.rows, 3, CV_64F);
        p2s = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
        p23D = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
        vector<vecDist> distancias;

        // Send images points to sphere model by generic camera model
        if (toSphere(matching_result.p1, matching_result.p2, p1s, p2s, state.params) < 0)
        {
                cout << "[ERROR] Error en toSphere" << endl;
                return -1;
        }

        // Calculate the distances between the points in the sphere
        // and sorting these distance for choose the greater ones
        distances(p1s, p2s, distancias, state.params);
        // sort(distancias.begin(), distancias.end(), mayorQue);

        // Choosing the gain for the control law
        float lambda = state.params.gainv;

        // Get interaction matrix and error vector with distances
        L = Lvl(p2s, distancias, state.params);
        Mat ERROR = Mat::zeros(distancias.size(), 1, CV_64F), ERROR_PIX = Mat::zeros(distancias.size(), 1, CV_64F);

        // for (int i = 0; i < 16; i++)
        for (int i = 0; i < distancias.size(); i++)
        {
                ERROR.at<double>(i, 0) = (double)distancias[i].dist2 - (double)distancias[i].dist;
                ERROR_PIX.at<double>(i, 0) = (double)norm(matching_result.p1.row(distancias[i].i) - matching_result.p2.row(distancias[i].i));
                // cout << i << " Distancia: " << distancias[i].dist << " Distancia2: " << distancias[i].dist2 << endl;
                // if ()
                // cout << "Pixeles: " << matching_result.p1.row(distancias[i].i) << " - " << matching_result.p2.row(distancias[i].j) << " -> " << norm(matching_result.p1.row(distancias[i].i) - matching_result.p2.row(distancias[i].j)) << endl;
        }

        // Mat a = Mat(matching_result.p1);
        // Mat b = Mat(matching_result.p2);
        // matching_result.mean_feature_error = norm(a, b) / ((double)matching_result.p1.rows);

        matching_result.mean_feature_error = norm(ERROR, NORM_L2);
        matching_result.mean_feature_error_pix = norm(ERROR_PIX, NORM_L2);

        cout << "[INFO] Error actual: " << matching_result.mean_feature_error << endl;

        // Mat a = Mat(matching_result.p1);
        // Mat b = Mat(matching_result.p2);
        // matching_result.mean_feature_error = norm(a, b) / ((double)matching_result.p1.rows);
        // Get the Penrose pseudo-inverse of the interaction matrix
        double det = 0.0;
        Lo = Moore_Penrose_PInv(L, det);
        if (det < 1e-6)
        {
                cout << "[ERROR] DET = ZERO --> det = " << det << endl;
                return -1;
        }

        // Get the control law with dimentions 3x1 in translation
        // if (matching_result.mean_feature_error > 0.1)
        // {
        //         U_temp = -lambda * Lo * ERROR;
        // }
        // else
        // {
        //         U_temp = -2 * lambda * Lo * ERROR;
        // }

        double l0 = 5 * lambda, linf = lambda, lprima = 1;
        double lambda_temp = (l0 - linf) * exp(-(lprima * matching_result.mean_feature_error) / (l0 - linf)) + linf;
        state.lambda_kp = lambda_temp;

        U_temp = -lambda_temp * Lo * ERROR;

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
        p1s.release();
        p2s.release();
        p23D.release();
        distancias.clear();

        return 0;
}

int toSphere(Mat p1,               // Points in the target image
             Mat p2,               // Points in the actual image
             Mat &p1s,             // Empty matrix for 3D recovery direction on sphere of p1 points
             Mat &p2s,             // Empty matrix for 3D recovery direction on sphere of p2 points
             vc_parameters &params // Parameters of the camera
)
{
        Mat temp = Mat::zeros(3, 1, CV_64F), tmp; // Temporal matrix for calculation

        for (int i = 0; i < p1.rows; i++)
        {
                // Take the points in target image and add 1 to the last row
                temp.at<double>(0, 0) = p1.at<double>(i, 0);
                temp.at<double>(1, 0) = p1.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                // Invert the matrix of the camera and multiply by the points
                tmp = params.K.inv() * temp;
                // Normalize the points
                p1s.row(i) = tmp.t() / norm(tmp);

                // Take the points in actual image and add 1 to the last row
                temp.at<double>(0, 0) = p2.at<double>(i, 0);
                temp.at<double>(1, 0) = p2.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                // Invert the matrix of the camera and multiply by the points
                tmp = params.K.inv() * temp;
                // Normalize the points
                p2s.row(i) = tmp.t() / norm(tmp);
        }
        // Free the memory
        temp.release();
        tmp.release();
        return 0;
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
                temp = s * ( (pi * ortoProj(pj)) + (pj * ortoProj(pi)) );
                temp.copyTo(L.row(i));
        }
        temp.release();
        pi.release();
        pj.release();
        return L;
}