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
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

image_geometry::PinholeCameraModel cam_model;
int frame_height;
int frame_width;
float centerDist;

class depth2cam
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber depthImageSub;
  image_transport::Subscriber rgbImageSub;
  ros::Subscriber depthPointsSub;
  ros::Subscriber camInfoSub;
  tf::TransformListener listener;

public:
  depth2cam() : it(nh)
  {
	depthImageSub = it.subscribe("camera/depth/image", 1,
		&depth2cam::depthImageCallback, this);
	rgbImageSub = it.subscribe("usb_cam/image_raw", 1,
		&depth2cam::rgbImageCallback, this);
	//depthPointsSub = nh.subscribe("camera/depth/points", 1,
	//	&depth2cam::depthPointsCallback, this);
	//camInfoSub = nh.subscribe("camera/depth/camera_info", 1,
	//	&depth2cam::camInfoCallback, this);

	namedWindow("depth view");
 	namedWindow("rgb view");
  }

  ~depth2cam()
  {
	destroyWindow("depth view");
	destroyWindow("rgb view");
  }

private:
//  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
//  {
//    cam_model.fromCameraInfo(info_msg);
//  }

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
	  tf::Matrix3x3 m(transform.getRotation());
	  double roll, pitch, yaw;
	  m.getRPY(roll, pitch, yaw);

	  float xdiff = transform.getOrigin().x() - centerDist;
	  float ydiff = transform.getOrigin().y();
	  float zdiff = transform.getOrigin().z();

	  float v_angle = atan2(zdiff, xdiff);
	  float h_angle = atan2(ydiff, xdiff);

	  float v_focal_length = 215.111;
	  float u_focal_length = 288.648;

	  int v = v_focal_length*tan(v_angle+pitch);
	  int u = u_focal_length*tan(h_angle+yaw);

	  int pixel_y = (frame_height/2) - 1 - v;
	  int pixel_x = (frame_width/2) - 1 + u;
cout << "pixel y: " << pixel_y << endl;
cout<<"pixel x: " << pixel_x<<endl;


	  circle(cv_ptr_rgb, Point(pixel_x, pixel_y), 2, Scalar(255,0,255), 2);
	}

  	imshow("rgb view", cv_ptr_rgb);
  	waitKey(1);
  }

/*  void depthPointsCallback(const PointCloud::ConstPtr& msg)
  {
  	PointCloud depth = *msg;
  	int frameHeight = msg->height;
  	int frameWidth = msg->width;
  	int centerIndex = 319*239; //frameHeight*frameWidth*0.25;
  	cout << depth.at(centerIndex) << endl;

  	geometry_msgs::Point depthXYZ;
  	depthXYZ.x = depth.at(centerIndex).x;
  	depthXYZ.y = depth.at(centerIndex).y;
  	depthXYZ.z = depth.at(centerIndex).z;
  	cout << depthXYZ << endl;
  	cout << " " << endl;

  	Point3d pt_cv(depthXYZ.x, depthXYZ.y, depthXYZ.z);
  	Point2d uv;
  	uv = cam_model.project3dToPixel(pt_cv);

	cout << uv << endl;
	cout << " " << endl;
  }*/
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth2cam_projection");

  ros::param::get("/usb_cam/image_width", frame_width);
  ros::param::get("/usb_cam/image_height", frame_height);

  depth2cam d2c;
  ros::spin();
  return 0;
}
