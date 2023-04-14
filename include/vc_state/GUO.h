#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int toSphere(Mat p1, Mat p2, Mat &p1s, Mat &p2s, vc_parameters &params);
int distances(Mat p1, Mat p2, vector<vecDist> &distancias, vc_parameters &params);
bool mayorQue(vecDist a, vecDist b);
Mat ortoProj(Mat p1);
Mat Lvl(Mat p2s, vector<vecDist> &distances, vc_parameters &params);

Mat signMat(Mat mat);
Mat robust(Mat error);