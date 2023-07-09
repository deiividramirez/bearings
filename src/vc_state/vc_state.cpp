#include "vc_state/vc_state.h"
using namespace std;

vc_state::vc_state() : X(0), Y(0), Z(0), Yaw(0), Pitch(0), Roll(0),
                       initialized(false), t(0), dt(0.025), Kv(1.0), Kw(1.0) {}

void vc_state::load(const ros::NodeHandle &nh)
{
        cout << "\n\n[INFO] Loading state parameters" << endl;

        // Load intrinsic parameters
        XmlRpc::XmlRpcValue kConfig;
        this->params.K = cv::Mat(3, 3, CV_64F, double(0));
        if (nh.hasParam("camera_intrinsic_parameters"))
        {
                nh.getParam("camera_intrinsic_parameters", kConfig);
                if (kConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
                        for (int i = 0; i < 9; i++)
                        {
                                std::ostringstream ostr;
                                ostr << kConfig[i];
                                std::istringstream istr(ostr.str());
                                istr >> this->params.K.at<double>(i / 3, i % 3);
                        }
        }
        cout << "[INFO] Calibration Matrix \n"
             << this->params.K << endl;

        this->params.Kinv = this->params.K.inv();

        this->R = cv::Mat(3, 3, CV_64F, double(0));
        if (nh.hasParam("R"))
        {
                nh.getParam("R", kConfig);
                if (kConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
                        for (int i = 0; i < 9; i++)
                        {
                                std::ostringstream ostr;
                                ostr << kConfig[i];
                                std::istringstream istr(ostr.str());
                                istr >> this->R.at<double>(i / 3, i % 3);
                        }
        }

        cout << "[INFO] Rotation Matrix \n"
             << this->R << endl;

        // Load error threshold parameter
        // Load feature detector parameters
        this->params.fastThreshold = nh.param(std::string("fastThreshold"), 20);
        this->params.nfeatures = nh.param(std::string("nfeatures"), 100);
        this->params.nlevels = nh.param(std::string("nlevels"), 5);
        this->params.edgeThreshold = nh.param(std::string("edgeThreshold"), 15);
        this->params.feature_threshold = nh.param(std::string("feature_error_threshold"), std::numeric_limits<double>::max());
        this->params.scaleFactor = nh.param(std::string("scaleFactor"), 1.0);
        this->params.patchSize = nh.param(std::string("patchSize"), 30);
        this->params.flann_ratio = nh.param(std::string("flann_ratio"), 0.7);
        
        this->params.control = nh.param(std::string("control"), 1);
        this->params.camara = nh.param(std::string("camara"), 1);

        this->Kv = nh.param(std::string("gainv"), 0.0);
        this->Kw = nh.param(std::string("gainw"), 0.0);
        this->Kv_max = nh.param(std::string("gainv_max"), 0.0);
        this->Kw_max = nh.param(std::string("gainw_max"), 0.0);

        XmlRpc::XmlRpcValue bearingConfig;
        if (nh.hasParam("bearing"))
        {
                nh.getParam("bearing", bearingConfig);
                this->params.bearing = cv::Mat((int)bearingConfig["rows"], (int)bearingConfig["cols"], CV_64F);
                for (int i = 0; i < (int)bearingConfig["rows"]; i++)
                {
                        for (int j = 0; j < (int)bearingConfig["cols"]; j++)
                        {
                                std::ostringstream ostr;
                                ostr << bearingConfig["data"][i * (int)bearingConfig["cols"] + j];
                                std::istringstream istr(ostr.str());
                                istr >> this->params.bearing.at<double>(i, j);
                        }
                }
        }

        XmlRpc::XmlRpcValue seguimientoConfig;
        if (nh.hasParam("seguimiento"))
        {
                nh.getParam("seguimiento", seguimientoConfig);
                this->params.seguimiento = cv::Mat((int)seguimientoConfig["len"], 1, CV_64F);
                for (int i = 0; i < (int)seguimientoConfig["len"]; i++)
                {
                        std::ostringstream ostr;
                        ostr << seguimientoConfig["data"][i];
                        std::istringstream istr(ostr.str());
                        istr >> this->params.seguimiento.at<double>(i, 0);
                }
        }

        // Load sampling time parameter
        this->dt = nh.param(std::string("dt"), 0.01);
}

std::pair<Eigen::VectorXd, float> vc_state::update()
{
        this->t += this->dt;
        this->X = this->X + this->Vx * this->dt;
        this->Y = this->Y + this->Vy * this->dt;
        this->Z = this->Z + this->Vz * this->dt;
        this->Yaw = this->Yaw + this->Vyaw * this->dt;

        // this->X = -3.5;
        // this->Y = -0.5;
        // this->Z = 2.2;
        // this->Yaw = 0;

        cout << endl
             << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
             << "SENDING THIS POS >>" << endl
             << "X: " << this->X << " - Y: " << this->Y << " - Z: " << this->Z << " - Yaw: " << this->Yaw << endl
             << "Vx: " << this->Vx << " - Vy: " << this->Vy << " - Vz: " << this->Vz << " - Vyaw: " << this->Vyaw << endl
             << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
             << endl;

        // ros::Duration(0.1).sleep();

        Eigen::VectorXd position;
        position.resize(3);

        position(0) = this->X;
        position(1) = this->Y;
        position(2) = this->Z;

        return make_pair(position, this->Yaw);
}

// std::vector<double> vc_state::position()
// {
//   std::vector<double> position;
//   position.push_back(this->X);
//   position.push_back(this->Y);
//   position.push_back(this->Z);
//   return position;
// }

void vc_state::initialize(const float &x, const float &y, const float &z, const float &yaw)
{
        this->X = x;
        this->Y = y;
        this->Z = z;
        this->Yaw = yaw;

        this->Vx = 0;
        this->Vy = 0;
        this->Vz = 0;
        this->Vyaw = 0;

        cout << "Init pose: X: " << this->X << " Y: " << this->Y << " Z: " << this->Z << endl;
        cout << "Yaw: " << this->Yaw << " Pitch: " << this->Pitch << " Roll: " << this->Roll << endl;

        this->initialized = true;
        // vc_state::update();
}

Mat signMat(Mat mat)
{
        Mat sign = Mat::zeros(mat.rows, 1, CV_64F);
        // get the sign of each element of mat
        // 1 if positive, -1 if negative and 0 if zero
        double tempsign;
        for (int i = 0; i < mat.rows; i++)
        {
                if (mat.at<double>(i, 0) > 0)
                {
                        tempsign = 1;
                }
                else if (mat.at<double>(i, 0) < 0)
                {
                        tempsign = -1;
                }
                else
                {
                        tempsign = 0;
                }
                sign.at<double>(i, 0) = tempsign;
        }
        return sign;
}

Mat robust(Mat error)
{
        Mat sign = signMat(error);
        Mat absError = abs(error), sqrtError;

        sqrt(absError, sqrtError);
        Mat robustError = sqrtError.mul(sign);

        return robustError;
}

Mat composeR(double roll, double pitch, double yaw)
{
        Mat R = Mat::zeros(3, 3, CV_64F);

        R.at<double>(0, 0) = cos(yaw) * cos(pitch);
        R.at<double>(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
        R.at<double>(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);

        R.at<double>(1, 0) = sin(yaw) * cos(pitch);
        R.at<double>(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
        R.at<double>(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

        R.at<double>(2, 0) = -sin(pitch);
        R.at<double>(2, 1) = cos(pitch) * sin(roll);
        R.at<double>(2, 2) = cos(pitch) * cos(roll);

        return R;
}

Mat composeR(Mat rvec)
{
        Mat R = Mat::zeros(3, 3, CV_64F);

        double roll = rvec.at<double>(0, 0);
        double pitch = rvec.at<double>(1, 0);
        double yaw = rvec.at<double>(2, 0);

        R.at<double>(0, 0) = cos(yaw) * cos(pitch);
        R.at<double>(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
        R.at<double>(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);

        R.at<double>(1, 0) = sin(yaw) * cos(pitch);
        R.at<double>(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
        R.at<double>(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

        R.at<double>(2, 0) = -sin(pitch);
        R.at<double>(2, 1) = cos(pitch) * sin(roll);
        R.at<double>(2, 2) = cos(pitch) * cos(roll);

        return R;
}

Mat decomposeR(Mat R)
{
        double roll, pitch, yaw;

        pitch = -asin(R.at<double>(2, 0));
        roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

        Mat euler = Mat::zeros(3, 1, CV_64F);
        euler.at<double>(0, 0) = roll;
        euler.at<double>(1, 0) = pitch;
        euler.at<double>(2, 0) = yaw;

        return euler;
}

Mat projOrtog(Mat &x)
{
        Mat Px = (Mat::eye(3, 3, CV_64F) - x * x.t());
        return Px;
}

Mat puntoMedio(Mat p1, Mat p2, Mat p3, Mat p4)
{
        if (p1.type() == CV_64F && p2.type() == CV_64F && p3.type() == CV_64F && p4.type() == CV_64F)
        {
                Mat pMedio = Mat::zeros(3, 1, CV_64F);
                pMedio.at<double>(0, 0) = (p1.at<double>(0, 0) + p2.at<double>(0, 0) + p3.at<double>(0, 0) + p4.at<double>(0, 0)) / 4;
                pMedio.at<double>(0, 1) = (p1.at<double>(0, 1) + p2.at<double>(0, 1) + p3.at<double>(0, 1) + p4.at<double>(0, 1)) / 4;
                pMedio.at<double>(0, 2) = (p1.at<double>(0, 2) + p2.at<double>(0, 2) + p3.at<double>(0, 2) + p4.at<double>(0, 2)) / 4;
                return pMedio;
        }
        else
        {
                Mat pMedio = Mat::zeros(3, 1, CV_32F);
                pMedio.at<float>(0, 0) = (p1.at<float>(0, 0) + p2.at<float>(0, 0) + p3.at<float>(0, 0) + p4.at<float>(0, 0)) / 4;
                pMedio.at<float>(0, 1) = (p1.at<float>(0, 1) + p2.at<float>(0, 1) + p3.at<float>(0, 1) + p4.at<float>(0, 1)) / 4;
                pMedio.at<float>(0, 2) = (p1.at<float>(0, 2) + p2.at<float>(0, 2) + p3.at<float>(0, 2) + p4.at<float>(0, 2)) / 4;
                return pMedio;
        }
}

Mat puntoMedio(Mat p1, Mat p2)
{
        if (p1.type() == CV_64F && p2.type() == CV_64F)
        {
                Mat pMedio = Mat::zeros(1, 2, CV_64F);
                pMedio.at<double>(0, 0) = (p1.at<double>(0, 0) + p2.at<double>(0, 0)) / 2;
                pMedio.at<double>(0, 1) = (p1.at<double>(0, 1) + p2.at<double>(0, 1)) / 2;
                return pMedio;
        }
        else
        {
                Mat pMedio = Mat::zeros(1, 2, CV_32F);
                pMedio.at<float>(0, 0) = (int)((p1.at<float>(0, 0) + p2.at<float>(0, 0)) / 2);
                pMedio.at<float>(0, 1) = (int)((p1.at<float>(0, 1) + p2.at<float>(0, 1)) / 2);
                return pMedio;
        }

        // Mat pMedio = Mat::zeros(2, 1, CV_64F);
        // pMedio.at<double>(0, 0) = (p1.at<double>(0, 0) + p2.at<double>(0, 0)) / 2;
        // pMedio.at<double>(1, 0) = (p1.at<double>(0, 1) + p2.at<double>(0, 1)) / 2;
        // return pMedio;
}

string type2str(int type)
{
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth)
        {
        case CV_8U:
                r = "8U";
                break;
        case CV_8S:
                r = "8S";
                break;
        case CV_16U:
                r = "16U";
                break;
        case CV_16S:
                r = "16S";
                break;
        case CV_32S:
                r = "32S";
                break;
        case CV_32F:
                r = "32F";
                break;
        case CV_64F:
                r = "64F";
                break;
        default:
                r = "User";
                break;
        }

        r += "C";
        r += (chans + '0');

        return r;
}

void Tipito(Mat &Matrix)
{
        string ty = type2str(Matrix.type());
        cout << "Matrix: " << ty.c_str() << " " << Matrix.cols << "x" << Matrix.rows << endl;
        cout << Matrix << endl;
}