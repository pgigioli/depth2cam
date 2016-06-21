#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

const double pi = 3.1415926535897;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.76) );
   tf::Quaternion q;
   //float pan_angleRAD = msg->data[0] * pi/180;
   float tilt_angleRAD = msg->data[1] * pi/180;
   //float t1 = pan_angleRAD;
   float offset = 0.03;
   float t2 = tilt_angleRAD;

   tf::Matrix3x3 tf3d;
   tf3d.setValue(cos(t2), 0.0, sin(t2),
        0.0, 1.0, 0.0,
        -sin(t2), 0.0, cos(t2));
   tf::Quaternion tfqt;
   tf3d.getRotation(tfqt);
   transform.setRotation(tfqt);
   transform.setOrigin(tf::Vector3(offset*cos(t2), 0.0, -offset*sin(t2)));
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
				"cam_base", "webcam"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cambase2webcam_tf_broadcaster");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cam_angles", 1, callback);
  ros::spin();
  return 0;
}
