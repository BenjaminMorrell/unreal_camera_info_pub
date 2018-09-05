#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

int main(int argc, char** argv)
{
  if (argv[1] == NULL) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/right/image_raw", 1);
  
  std_msgs::Header header;
  ros::Time timestamp;

  

  std::string filestem = argv[1];

  std::string extension;

  if (argc <3){
    extension = ".png";
  } else{
    extension = "." + SSTR(argv[2]);
  }
  
  
  std::string filename;

  int tickCount = 0;

  ros::Rate loop_rate(1);

  cv::Mat image;
  sensor_msgs::ImagePtr msg;

  while (nh.ok() && tickCount < 9)
  {
    
    filename = filestem + SSTR(tickCount) + extension;

    std::cout << "filename is: " << filename << std::endl;

    image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    
    header.seq = ++tickCount;
    
    header.stamp = timestamp.now();
    header.frame_id = "dummy_frame";
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
