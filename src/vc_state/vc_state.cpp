#include "vc_state/vc_state.h"
using namespace std;

vc_state::vc_state() : X(0), Y(0), Z(0), Yaw(0), Pitch(0), Roll(0),
                       initialized(false), t(0), dt(0.025), Kv(1.0), Kw(1.0) {}

void vc_state::load(const ros::NodeHandle &nh)
{
        cout << "[INFO] Loading state parameters" << endl;

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

        // Load error threshold parameter
        this->params.feature_threshold = nh.param(std::string("feature_error_threshold"), std::numeric_limits<double>::max());
        // Load feature detector parameters
        this->params.nfeatures = nh.param(std::string("nfeatures"), 100);
        this->params.scaleFactor = nh.param(std::string("scaleFactor"), 1.0);
        this->params.nlevels = nh.param(std::string("nlevels"), 5);
        this->params.edgeThreshold = nh.param(std::string("edgeThreshold"), 15);
        this->params.patchSize = nh.param(std::string("patchSize"), 30);
        this->params.fastThreshold = nh.param(std::string("fastThreshold"), 20);
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
        // Integrating
        this->X = this->X + this->Vx * this->dt;
        this->Y = this->Y + this->Vy * this->dt;
        this->Z = this->Z + this->Vz * this->dt;
        this->Yaw = this->Yaw + this->Vyaw * this->dt;

        cout << endl
             << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
             << "SENDING THIS POS >>" << endl 
             << "Vx: " << this->Vx << " - Vy: " << this->Vy << " - Vz: " << this->Vz << " - Vyaw: " << this->Vyaw << endl
             << "X: " << this->X << " - Y: " << this->Y << " - Z: " << this->Z << " - Yaw: " << this->Yaw << endl
             << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
             << endl;

        // ros::Duration(0.1).sleep();

        Eigen::VectorXd position;
        position.resize(3);

        position(0) = this->X;
        position(1) = this->Y;
        position(2) = this->Z;

        /* this->Vx = 0;
        this->Vy = 0;
        this->Vz = 0;
        this->Vyaw = 0; */

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
        this->initialized = true;
        cout << "Init pose: X: " << this->X << " Y: " << this->Y << " Z: " << this->Z << endl;
        cout << "Yaw: " << this->Yaw << " Pitch: " << this->Pitch << " Roll: " << this->Roll << endl;
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