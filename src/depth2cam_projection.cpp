#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>

using namespace std;

void CVcallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << endl;

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        // imshow expects a float value to lie in [0,1], so we need to normalize
        // for visualization purposes.
        double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        cv::Mat normalized;
        cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0)  ;

        cv::imshow("depth2cam", normalized);
        cv::waitKey(1);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  sensor_msgs::PointCloud2 depth;
printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth2cam_projection");

  ros::NodeHandle nh;
  ros::Subscriber OpenCVsub = nh.subscribe("camera/depth/image_raw", 1, CVcallback);
  ros::Subscriber depthSub = nh.subscribe("camera/depth/points", 1, depthCallback);

  cv::namedWindow("depth2cam");
  ros::spin();
  cv::destroyWindow("depth2cam");

  return 0;
}
