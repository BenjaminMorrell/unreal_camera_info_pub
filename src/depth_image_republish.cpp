#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/depth_registered/image", 1);
  }

  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::CV_16UC1);
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    try
    {
      // cv_ptr = cv_bridge::toCvShare(cv_ptr->toImageMsg(), sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr = cv_bridge::toCvShare(cv_ptr->toImageMsg(), sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception when trying to convert: %s", e.what());
      return;
    }

    
    // Output modified image
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}