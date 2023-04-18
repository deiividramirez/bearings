#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0, c7 = 0, c8 = 0, c9 = 0, c10 = 0;

void saveDesired1f(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 1f" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 1f received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_1f.jpg";
   imwrite(saveIMG, actual);
   if (c1++ == 1)
   cout << "[INFO] << Image 1f saved >> " << saveIMG << endl;

}
void saveDesired1b(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 1b" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 1b received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_1b.jpg";
   imwrite(saveIMG, actual);
   if (c2++ == 1)
   cout << "[INFO] << Image 1b saved >> " << saveIMG << endl;
}

void saveDesired2f(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 2f" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 2f received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_2f.jpg";
   imwrite(saveIMG, actual);
   if (c3++ == 1)
   cout << "[INFO] << Image 2f saved >> " << saveIMG << endl;
}
void saveDesired2b(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 2b" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 2b received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_2b.jpg";
   imwrite(saveIMG, actual);
   if (c4++ == 1)
   cout << "[INFO] << Image 2b saved >> " << saveIMG << endl;
}

void saveDesired3f(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 3f" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 3f received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_3f.jpg";
   imwrite(saveIMG, actual);
   if (c5++ == 1)
   cout << "[INFO] << Image 3f saved >> " << saveIMG << endl;
}
void saveDesired3b(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 3b" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 3b received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_3b.jpg";
   imwrite(saveIMG, actual);
   if (c6++ == 1)
   cout << "[INFO] << Image 3b saved >> " << saveIMG << endl;
}

void saveDesired4f(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 4f" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 4f received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_4f.jpg";
   imwrite(saveIMG, actual);
   if (c7++ == 1)
   cout << "[INFO] << Image 4f saved >> " << saveIMG << endl;
}
void saveDesired4b(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 4b" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 4b received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_4b.jpg";
   imwrite(saveIMG, actual);
   if (c8++ == 1)
   cout << "[INFO] << Image 4b saved >> " << saveIMG << endl;
}

void saveDesired5f(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 4f" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 4f received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_5f.jpg";
   imwrite(saveIMG, actual);
   if (c9++ == 1)
   cout << "[INFO] << Image 5f saved >> " << saveIMG << endl;
}
void saveDesired5b(const sensor_msgs::Image::ConstPtr &msg)
{
   /* cout << "[INFO] Saving Desired 4b" << endl; */

   Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
   // cout << "[INFO] Image 4b received" << endl;

   string saveIMG = WORKSPACE;
   saveIMG += + "/src/bearings/src/desired_5b.jpg";
   imwrite(saveIMG, actual);
   if (c10++ == 1)
   cout << "[INFO] << Image 4b saved >> " << saveIMG << endl;
}