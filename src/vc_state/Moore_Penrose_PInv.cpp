#include "vc_state/vc_state.h"

using namespace cv;

Mat Moore_Penrose_PInv(Mat L, double &det)
{
    Mat Lt = L.t();
    Mat Ls = Lt * L;
    
    det = determinant(Ls);

    if (det > 1e-8)
    {
        return Ls.inv() * Lt;
    }
    else
    {
        cout << "Moore_Penrose_PInv: Singular matrix" << endl;
    }

    return Lt;
}
