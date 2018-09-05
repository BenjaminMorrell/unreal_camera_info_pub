#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

int main(int argc, char** argv)
{
  if (argv[1] == NULL) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/right/image_raw", 1);

  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  

  int tickCount = 0;

  ros::Rate loop_rate(1);
  
  while (nh.ok())
  {

    std_msgs::Header header;
    header.seq = ++tickCount;
    ros::Time timestamp;
    header.stamp = timestamp.now();
    header.frame_id = "dummy_frame";
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

