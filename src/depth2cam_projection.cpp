#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

const double pi = 3.1415926535897;
int FRAME_H;
int FRAME_W;
float centerDist;
vector<vector<float> > DEPTH_LINE;
int LINE_LENGTH = 80;
float PAN_RAD;
float TILT_RAD;

class depth2cam
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber depthImageSub;
  image_transport::Subscriber rgbImageSub;
  ros::Subscriber cam_angles_sub;
  tf::TransformListener listener;

public:
  depth2cam() : it(nh)
  {
	depthImageSub = it.subscribe("camera/depth/image", 1,
		&depth2cam::depthImageCallback, this);
	rgbImageSub = it.subscribe("usb_cam/image_raw", 1,
		&depth2cam::rgbImageCallback, this);
	cam_angles_sub = nh.subscribe("/cam_angles", 1,
		&depth2cam::camAnglesCallback, this);

	namedWindow("depth view");
 	namedWindow("rgb view");
  }

  ~depth2cam()
  {
	destroyWindow("depth view");
	destroyWindow("rgb view");
  }

private:
  void camAnglesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
	PAN_RAD = msg->data[0] * pi/180;
	TILT_RAD = msg->data[1] * pi/180;
	return;
  }

  void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
  	try
	{
	  cv_bridge::CvImageConstPtr cv_ptr_depth;
	  cv_ptr_depth = cv_bridge::toCvShare(msg);

          // imshow expects a float value to lie in [0,1], so we need to normalize
          // for visualization purposes.
          double max = 0.0;
          minMaxLoc(cv_ptr_depth->image, 0, &max, 0, 0);
          Mat normalized;
          cv_ptr_depth->image.convertTo(normalized, CV_32F, 1.0/max, 0);

	  int width = cv_ptr_depth->image.cols;
	  int height = cv_ptr_depth->image.rows;
	  int centerX = (width * 0.5) - 1;
	  int centerY = (height * 0.5) - 1;

	  centerDist = cv_ptr_depth->image.at<float>( centerY, centerX );
	  //cout << centerDist << endl;
//for (int i = 0; i < LINE_LENGTH; i++) {
//int x = (width*0.5) - 1 - LINE_LENGTH/2 + i;
//int y = (height*0.5) - 1;
//float coordx = atan2(LINE_LENGTH/2 - i, 288.0);
//float coordy = 0.0;
//vector<float> coords;
//coords.push_back(coordx);
//coords.push_back(coordy);
//coords.push_back(cv_ptr_depth->image.at<float>(y, x));
//DEPTH_LINE.push_back(coords);
//coords.clear();

//circle(normalized, Point(x, y), 2, (0,255,0), 2);
//circle(normalized, Point(x, y), 1, 255, 1);
//}

	  circle(normalized, Point(centerX, centerY), 2, (0,255,0), 2);
	  circle(normalized, Point(centerX, centerY), 1, 255, 1);

          imshow("depth view", normalized);
          waitKey(1);
  	}
	catch (const cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
  	}
  }

  void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    	cv_bridge::CvImagePtr cv_ptr_bridge;
    	Mat cv_ptr_rgb;

    	try
	{
	  cv_ptr_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  cv_ptr_rgb = cv_ptr_bridge->image.clone();
  	}
	catch (const cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	tf::StampedTransform transform;

  	try
	{
    	  listener.lookupTransform("/camera_depth_optical_frame", "/webcam",
						 ros::Time(0), transform);
  	}
	catch (tf::TransformException ex)
	{
	  ROS_ERROR("%s",ex.what());
  	  ros::Duration(1.0).sleep();
  	}

	if (isnan(centerDist) == 0)
	{
//for (int i = 0; i < LINE_LENGTH; i++) {
	  geometry_msgs::PointStamped depthPoint, depthPoint2rgb;
	  depthPoint.header.seq = 0;
	  depthPoint.header.stamp = ros::Time(0);
	  depthPoint.header.frame_id = "camera_depth_optical_frame";
//vector<float> vector = DEPTH_LINE[i];
	  depthPoint.point.x = centerDist;
	  depthPoint.point.y = 0.0;
	  depthPoint.point.z = 0.0;
//depthPoint.point.z = vector[2];
//depthPoint.point.x = vector[0];
//depthPoint.point.y = vector[1];
	  listener.transformPoint("/webcam", depthPoint, depthPoint2rgb);
cout << depthPoint2rgb << endl;
	  tf::Matrix3x3 m(transform.getRotation());
	  double roll, pitch, yaw;
	  m.getRPY(roll, pitch, yaw);

float xdiff = depthPoint2rgb.point.x;
float ydiff = depthPoint2rgb.point.y;
float zdiff = depthPoint2rgb.point.z;
cout << "depth point: " << xdiff << ", " << ydiff << ", " << zdiff << endl;
	  float v_angle = atan2(zdiff, xdiff) + TILT_RAD;// - 0.052399;
	  float h_angle = atan2(ydiff, xdiff);
cout << "v angle: " << v_angle << endl;
cout << "h angle: " << h_angle << endl;
	  float focal_length = 292.0; //292

	  int v = focal_length*tan(v_angle);
	  int u = focal_length*tan(h_angle);
cout << "v: " << v << endl;
cout << "u: " << u << endl;
	  int pixel_y = (FRAME_H/2) - 1 - v;
	  int pixel_x = (FRAME_W/2) - 1 - u;
cout << "pixel y: " << pixel_y << endl;
cout << "pixel x: " << pixel_x<<endl;


	  circle(cv_ptr_rgb, Point(pixel_x, pixel_y), 2, Scalar(255,0,255), 2);
	}

  	imshow("rgb view", cv_ptr_rgb);
  	waitKey(1);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth2cam_projection");

  ros::param::get("/usb_cam/image_width", FRAME_W);
  ros::param::get("/usb_cam/image_height", FRAME_H);

  depth2cam d2c;
  ros::spin();
  return 0;
}
