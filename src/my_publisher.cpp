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

  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  if (!(video_sourceCmd >> video_source)) return 1;

  cv::VideoCapture cap(video_source);

  if (!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  int tickCount = 0;

  ros::Rate loop_rate(5);
  while (nh.ok())
  {
    cap >> frame;
      
    if (!frame.empty())
    {
      std_msgs::Header header;
      header.seq = ++tickCount;
      ros::Time timestamp;
      header.stamp = timestamp.now();
      header.frame_id = "dummy_frame";
      msg = cv_bridge::CvImage(header, "bgr8",frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    // cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // ros::Rate loop_rate(5);
    // while (nh.ok()) {
    //  pub.publish(msg);
    //  ros::spinOnce();
    //  loop_rate.sleep();
  }
}

