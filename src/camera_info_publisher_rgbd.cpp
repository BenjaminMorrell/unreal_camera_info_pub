#include "boost/bind.hpp"
#include <boost/thread/mutex.hpp>
#include <string>
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

class CameraInfoPublisher
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_left_sub_;
    ros::Publisher camera_left_info_pub_; 
    camera_info_manager::CameraInfoManager cinfo_l_;
  

public:
    CameraInfoPublisher(int argc, char** argv)
        : it_(nh_),
        image_left_sub_(it_.subscribe("/camera/depth_registered/image_raw",1, boost::bind(&CameraInfoPublisher::imageCb, this, _1, 1))),
		cinfo_l_(nh_)
	{
        camera_left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info",1000);
        
        // Initialise left camera
        std::string camera_name = "unreal_left";

        int option = 0;
        if (argc > 1){
            option = atoi(argv[1]);
        }
        
        std::string camera_info_url;
        if (option == 0){
            camera_info_url = "package://unreal_camera_info_pub/cfg/camera_left_info.yaml";
        }else{
            camera_info_url = "package://unreal_camera_info_pub/cfg/camera_left_info_high_res.yaml";
        }
        cinfo_l_.setCameraName(camera_name);
        cinfo_l_.validateURL(camera_info_url);
        cinfo_l_.loadCameraInfo(camera_info_url);

        
   }

    ~CameraInfoPublisher()
    {}

    
    void imageCb(const sensor_msgs::ImageConstPtr& im_msg, int id)
    {
        ROS_INFO("In image callback for CameraInfoPublisher");
        // Create camera info message - load from cinfo
        // Create a pointer
        sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo());

        // Load from the appropriate configuration
        if (id == 1){
        	msg.reset(new sensor_msgs::CameraInfo(cinfo_l_.getCameraInfo()));
        }
        
        // Set frame id and time stamp from the image
        msg->header.frame_id = im_msg->header.frame_id;
        msg->header.stamp = im_msg->header.stamp;


        ROS_INFO("Image info Message filled");

        // Publish
        if (id == 1){
            camera_left_info_pub_.publish(msg);
        }
    

    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher_rgbd");
    CameraInfoPublisher ic(argc, argv);
    ROS_INFO("completed initialisation");
    ros::spin();
    return 0;
}

