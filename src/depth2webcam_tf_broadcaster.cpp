#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

const double pi = 3.1415926535897;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
   static tf::TransformBroadcaster br;
   tf::Transform transform;
//   tf::Quaternion q;
//   float pan_angleRAD = msg->data[0] * pi/180;
//   float tilt_angleRAD = msg->data[1] * pi/180;

float th_p = msg->data[0] * pi/180;
float th_t = msg->data[1] * pi/180;

float px1 = 0.095;
float py1 = -0.04;
float pz1 = 0.33;
float a = 0.02;

float pxt = cos(th_t)*(px1*cos(th_p)-py1*sin(th_p)) + sin(th_t)*pz1 + a*cos(th_t);
float pyt = px1*sin(th_p) + py1*cos(th_p);
float pzt = -sin(th_t)*(px1*cos(th_p)-py1*sin(th_p)) + cos(th_t)*pz1 - a*sin(th_t);

   tf::Matrix3x3 tf3d;
   tf3d.setValue(cos(th_t)*cos(th_p), -cos(th_t)*sin(th_p), sin(th_t),
        sin(th_p), cos(th_p), 0.0,
        -sin(th_t)*cos(th_p), sin(th_t)*sin(th_p), cos(th_t));
   tf::Quaternion tfqt;
   tf3d.getRotation(tfqt);
   transform.setRotation(tfqt);

transform.setOrigin(tf::Vector3(pxt, pyt, pzt));
//q.setRPY(0, 0, pan_angleRAD);
//   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
				"camera_depth_optical_frame", "webcam"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth2webcam_tf_broadcaster");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cam_angles", 1, callback);
  ros::spin();
  return 0;
}
