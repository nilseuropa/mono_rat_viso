#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

nav_msgs::Odometry odom;
tf::Quaternion q;
std::string base_footprint_frame;
tf::TransformBroadcaster* odom_broadcaster = NULL;
ros::Publisher odom_pub;
ros::Time current_time;
ros::Time last_time;

std::vector<double> translation_intensity_profile;
std::vector<double> prev_translation_intensity_profile;
std::vector<double> rotation_intensity_profile;
std::vector<double> prev_rotation_intensity_profile;

geometry_msgs::Point position;

int image_width;

int trans_roi_x_min;
int trans_roi_x_max;
int trans_roi_y_min;
int trans_roi_y_max;

int rot_roi_x_min;
int rot_roi_x_max;
int rot_roi_y_min;
int rot_roi_y_max;

bool gotCamInfo = false;
bool gotImage   = false;
bool gotOdom    = false;
bool broadcast_tf;
bool cal_odom_scale;
bool reset_by_cmd_vel;

double yaw;

double image_freq;
double vtrans_scale, vtrans_lin_scale, vtrans_rot_scale;
double camera_fov_degrees;
double prev_linear_velocity, prev_angular_velocity;
double linear_velocity, angular_velocity;
double linear_highpass_cutoff, linear_lowpass_cutoff;
double angular_highpass_cutoff, angular_lowpass_cutoff;
double linear_lowpass_coefficient, angular_lowpass_coefficient;
double max_angular_vel, max_linear_vel, reset_lin_vel;

float pose_covariance[] = {
  0.001,  0,     0,         0,          0,          0,
  0,      0.001, 0,         0,          0,          0,
  0,      0,     1000000.0, 0,          0,          0,
  0,      0,     0,         1000000.0,  0,          0,
  0,      0,     0,         0,          1000000.0,  0,
  0,      0,     0,         0,          0,          0.03
};

void image_to_profile(double *current_profile, const unsigned char *image, bool grayscale, int X_RANGE_MIN, int X_RANGE_MAX, int Y_RANGE_MIN, int Y_RANGE_MAX)
{
  unsigned int TEMPLATE_Y_SIZE = 1;
  unsigned int TEMPLATE_X_SIZE = X_RANGE_MAX - X_RANGE_MIN;

  int data_next = 0;
  for (int i = 0; i < TEMPLATE_X_SIZE; i++)
  {
    current_profile[i] = 0;
  }

  int sub_range_x = X_RANGE_MAX - X_RANGE_MIN;
  int sub_range_y = Y_RANGE_MAX - Y_RANGE_MIN;
  int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
  int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
  int pos;

  if (grayscale)
  {
    for (int y_block = Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * image_width);
            current_profile[data_next] += (double)(image[pos]);
          }
        }
        current_profile[data_next] /= (255.0);
        current_profile[data_next] /= (x_block_size * y_block_size);
        data_next++;
      }
    }
  }
  else
  {
    for (int y_block = Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * image_width) * 3;
            current_profile[data_next] += ((double)(image[pos]) + (double)(image[pos + 1]) + (double)(image[pos + 2]));
          }
        }
        current_profile[data_next] /= (255.0 * 3.0);
        current_profile[data_next] /= (x_block_size * y_block_size);
        data_next++;
      }
    }
  }
}

void visual_odo(double *current_profile, unsigned short width, double *previous_profile, double *vtrans_ms, double *vrot_rads, double *vtrans_scale)
{
  double mindiff = 1e6;
  double minoffset = 0;
  double cdiff;
  int offset;
  int intensity_profile_length = width;
  int offset_range = 40;
  int k;

  for (offset = 0; offset < offset_range; offset++) {
    cdiff = 0;

    for (k = 0; k < intensity_profile_length - offset; k++) {
      cdiff += fabs(current_profile[k] - previous_profile[k + offset]);
    }

    cdiff /= (1.0 * (intensity_profile_length - offset));

    if (cdiff < mindiff) {
      mindiff = cdiff;
      minoffset = -offset;
    }
  }

  for (offset = 0; offset < offset_range; offset++) {
    cdiff = 0;

    for (k = 0; k < intensity_profile_length - offset; k++) {
      cdiff += fabs(current_profile[k + offset] - previous_profile[k]);
    }

    cdiff /= (1.0 * (intensity_profile_length - offset));

    if (cdiff < mindiff) {
      mindiff = cdiff;
      minoffset = offset;
    }
  }

  for (unsigned int i = 0; i < width; i++) {
    previous_profile[i] = current_profile[i];
  }

  // TODO: reverse translation
  *vrot_rads = minoffset * camera_fov_degrees / image_width * image_freq * M_PI / 180.0;
  *vtrans_ms = mindiff * *vtrans_scale;
}

geometry_msgs::Twist cmd_vel;
void cmdCallback(const geometry_msgs::Twist &c){
  cmd_vel = c;
}

nav_msgs::Odometry reference_odom;
void odomCallBack(const nav_msgs::Odometry &odom){
  reference_odom = odom;
  gotOdom = true;
}

void camInfoCallback(const sensor_msgs::CameraInfo info)
{
  image_width = info.width;
  gotCamInfo = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  if (dt>10) dt = 0.01; // HACK: fix initial dt error
  image_freq = 1.0/dt;
  last_time = current_time;

  double irrelevant, translation, rotation;
  // convert image region to horizontal intensity profile
  image_to_profile(&translation_intensity_profile[0], &image->data[0],  (image->encoding == "bgr8" ? false : true), trans_roi_x_min, trans_roi_x_max, trans_roi_y_min, trans_roi_y_max);
  visual_odo(&translation_intensity_profile[0], translation_intensity_profile.size(), &prev_translation_intensity_profile[0], &translation, &irrelevant, &vtrans_scale);
  // convert image region to vertical intensity profile
  image_to_profile(&rotation_intensity_profile[0], &image->data[0],  (image->encoding == "bgr8" ? false : true), rot_roi_x_min, rot_roi_x_max, rot_roi_y_min, rot_roi_y_max);
  visual_odo(&rotation_intensity_profile[0], rotation_intensity_profile.size(), &prev_rotation_intensity_profile[0], &irrelevant, &rotation, &vtrans_scale);

  // calculate filter coefficients
  if (dt > 0) {
    linear_lowpass_coefficient   = 1-exp(-dt * linear_lowpass_cutoff);
    angular_lowpass_coefficient  = 1-exp(-dt * angular_lowpass_cutoff);
  }
  // lowpass
  angular_velocity += (rotation - angular_velocity) * angular_lowpass_coefficient;
  linear_velocity  += (translation - linear_velocity) * linear_lowpass_coefficient;
  // highpass
  if (linear_velocity < linear_highpass_cutoff) linear_velocity = 0;
  // constrains
  if (linear_velocity > max_linear_vel) linear_velocity = max_linear_vel;

  // change scale on turns as translation is sensitive to rotation
  if (fabs(angular_velocity)>=0) vtrans_scale = vtrans_rot_scale; else vtrans_scale = vtrans_lin_scale;

  // stop integration if no command issued
  if (reset_by_cmd_vel){
    if ( fabs(cmd_vel.linear.x)<=reset_lin_vel ){
      linear_velocity = 0;
    }
    if ( cmd_vel.linear.x<0 ){ // the optical method does not support reversing yet
      linear_velocity *= -1.0;
    }
  }

  // integrating heading
  yaw += (angular_velocity+prev_angular_velocity) * dt/2;
  prev_angular_velocity = angular_velocity;
  q = tf::createQuaternionFromYaw(yaw);

  // integrating forward position
  double ds = (linear_velocity+prev_linear_velocity) * dt/2;
  prev_linear_velocity = linear_velocity;

  // pose update
  double cosTheta = cos(yaw);
  double sinTheta = sin(yaw);
  position.x += cosTheta*ds;
  position.y += sinTheta*ds;

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = base_footprint_frame;
  odom.twist.twist.linear.x  = linear_velocity;
  odom.twist.twist.angular.z = angular_velocity;
  odom.pose.pose.position    = position;
  tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
  for (uint8_t i = 0; i < 36; i++){ odom.pose.covariance[i] = pose_covariance[i];}
  odom_pub.publish(odom);

  if (broadcast_tf){
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id  = base_footprint_frame;
    odom_tf.transform.translation.x = position.x;
    odom_tf.transform.translation.y = position.y;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    odom_broadcaster->sendTransform(odom_tf);
  }

  gotImage = true;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "rat_o_meter");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  std::string camera_topic, camera_info;
  nhLocal.param("camera_rectified_topic", camera_topic, std::string("/head_camera/image_rect"));
  nhLocal.param("camera_info_topic", camera_info, std::string("/head_camera/camera_info"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber camera = it.subscribe(camera_topic, 2, imageCallback);

  odom_broadcaster = new(tf::TransformBroadcaster);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/viso/odom", 10);
  ros::Subscriber cmd_sub;

  nhLocal.param("base_footprint_frame", base_footprint_frame, std::string("rat_o_meter"));

  nhLocal.param("linear_highpass_cutoff",  linear_highpass_cutoff,  5.0);
  nhLocal.param("linear_lowpass_cutoff",   linear_lowpass_cutoff,   15.0);
  nhLocal.param("angular_highpass_cutoff", angular_highpass_cutoff, 10.0); // not in use
  nhLocal.param("angular_lowpass_cutoff",  angular_lowpass_cutoff,  15.0);

  nhLocal.param("camera_fov_degrees", camera_fov_degrees, 75.0); // PS3

  nhLocal.param("image_width",        image_width, 640);

  nhLocal.param("trans_roi_x_min",    trans_roi_x_min, 0);
  nhLocal.param("trans_roi_x_max",    trans_roi_x_max, 640);
  nhLocal.param("trans_roi_y_min",    trans_roi_y_min, 0);
  nhLocal.param("trans_roi_y_max",    trans_roi_y_max, 480);

  nhLocal.param("rot_roi_x_min",      rot_roi_x_min, 0);
  nhLocal.param("rot_roi_x_max",      rot_roi_x_max, 640);
  nhLocal.param("rot_roi_y_min",      rot_roi_y_min, 0);
  nhLocal.param("rot_roi_y_max",      rot_roi_y_max, 480);

  nhLocal.param("vtrans_lin_scale",   vtrans_lin_scale, 75.0);
  nhLocal.param("vtrans_rot_scale",   vtrans_rot_scale, 25.0);

  nhLocal.param("max_linear_vel",     max_linear_vel,  0.35);
  nhLocal.param("max_angular_vel",    max_angular_vel, 2.0);

  nhLocal.param("broadcast_tf",   broadcast_tf,   false);
  nhLocal.param("cal_odom_scale", cal_odom_scale, false);

  nhLocal.param("reset_lin_vel",    reset_lin_vel, 0.05);
  nhLocal.param("reset_by_cmd_vel", reset_by_cmd_vel, true);
  if (reset_by_cmd_vel) cmd_sub = nh.subscribe("/cmd_vel", 100, cmdCallback);

  std::string cal_odom_topic;
  ros::Subscriber ref_odom_sub;
  nhLocal.param("cal_odom_topic", cal_odom_topic, std::string("/odom"));

  if (cal_odom_scale) {
    ROS_INFO("Waiting for odometry...");
    ref_odom_sub = nh.subscribe(cal_odom_topic, 1, odomCallBack);
    while (nh.ok()&&!gotOdom) {
      ros::spinOnce();
    }
    position = reference_odom.pose.pose.position;
    tf::quaternionMsgToTF(reference_odom.pose.pose.orientation, q);
    yaw = getYaw(q);
  }

  bool subscribe_to_info;
  ros::Subscriber camInfoSub;
  nhLocal.param("subscribe_to_info",subscribe_to_info,true);
  if (subscribe_to_info){
    ROS_INFO("Waiting for camera_info...");
    camInfoSub = nh.subscribe(camera_info, 1, camInfoCallback);
    while (nh.ok()&&!gotCamInfo) {
      ros::spinOnce();
    }
  }

  translation_intensity_profile.resize(trans_roi_x_max-trans_roi_x_min);
  prev_translation_intensity_profile.resize(trans_roi_x_max-trans_roi_x_min);
  rotation_intensity_profile.resize(rot_roi_x_max-rot_roi_x_min);
  prev_rotation_intensity_profile.resize(rot_roi_x_max-rot_roi_x_min);

  ROS_INFO("Waiting for first image...");
  while (nh.ok()&&!gotImage) {
    ros::spinOnce();
  }

  for (unsigned int i = 0; i < translation_intensity_profile.size(); i++) {
    prev_translation_intensity_profile[i] = translation_intensity_profile[i];
  }
  for (unsigned int i = 0; i < rotation_intensity_profile.size(); i++) {
    prev_rotation_intensity_profile[i] = rotation_intensity_profile[i];
  }

  ROS_INFO("Rat-o-meter active...");
  ros::spin();
  return 0;
}
