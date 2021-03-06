#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

float pan_angle;
float tilt_angle;
std_msgs::Float64MultiArray anglesArray;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_cam_angles");

  pan_angle = 0.0;
  tilt_angle = 0.0;
  anglesArray.data.push_back(pan_angle);
  anglesArray.data.push_back(tilt_angle);

  ros::NodeHandle nh;
  ros::Publisher cam_angles = nh.advertise<std_msgs::Float64MultiArray>("/cam_angles", 1);

  while (ros::ok())
  {
	cam_angles.publish(anglesArray);
	ros::spinOnce();
  }
  return 0;
}
