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
  // this->params.gainv = nh.param(std::string("gainv"), 1.0);
  // this->params.gainw = nh.param(std::string("gainw"), 1.0);

  // Load gain parameters
  this->Kv = nh.param(std::string("gainv"), 0.0);
  this->Kw = nh.param(std::string("gainw"), 0.0);
  this->Kv_max = nh.param(std::string("gainv_max"), 0.0);
  this->Kw_max = nh.param(std::string("gainw_max"), 0.0);

  // Load sampling time parameter
  this->dt = nh.param(std::string("dt"), 0.01);
}

std::pair<Eigen::VectorXd, float> vc_state::update()
{
  this->t += this->dt;
  // Integrating
  this->X = this->X + this->Kv * this->Vx * this->dt;
  this->Y = this->Y + this->Kv * this->Vy * this->dt;
  this->Z = this->Z + this->Kv * this->Vz * this->dt;
  // this->Yaw = this->Yaw + this->Kw * this->Vyaw * this->dt;
  this->Yaw = this->Yaw - this->Kw * this->Yaw * this->dt;
  cout << endl
       << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
       << "SENDING THIS POS >> X: " << this->X << " Y:" << this->Y << " Z:" << this->Z << endl
       << "Vx: " << this->Vx << " Vy:" << this->Vy << " Vz:" << this->Vz << endl;

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
