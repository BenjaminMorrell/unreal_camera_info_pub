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
    image_transport::Subscriber image_right_sub_;
    ros::Publisher camera_left_info_pub_; 
    ros::Publisher camera_right_info_pub_; 
    camera_info_manager::CameraInfoManager cinfo_l_;
    camera_info_manager::CameraInfoManager cinfo_r_;

public:
    CameraInfoPublisher()
        : it_(nh_),
        image_left_sub_(it_.subscribe("/camera/left/image_raw",1, boost::bind(&CameraInfoPublisher::imageCb, this, _1, 1))),
        image_right_sub_(it_.subscribe("/camera/right/image_raw",1, boost::bind(&CameraInfoPublisher::imageCb, this, _1, 0))),
		cinfo_l_(nh_),
		cinfo_r_(nh_)
	{
        camera_left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/left/camera_info",1000);
        camera_right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/right/camera_info",1000);

        // Initialise left camera
        std::string camera_name = "unreal_left";
        
        std::string camera_info_url = "package://unreal_camera_info_pub/cfg/camera_left_info.yaml";
        cinfo_l_.setCameraName(camera_name);
        cinfo_l_.validateURL(camera_info_url);
        cinfo_l_.loadCameraInfo(camera_info_url);

        // Initialise right camera
        camera_name = "unreal_right";
        
		camera_info_url = "package://unreal_camera_info_pub/cfg/camera_right_info.yaml";
		cinfo_r_.setCameraName(camera_name);
		cinfo_r_.validateURL(camera_info_url);
		cinfo_r_.loadCameraInfo(camera_info_url);

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
        }else{
        	msg.reset(new sensor_msgs::CameraInfo(cinfo_r_.getCameraInfo()));
        }
        
        // Set frame id and time stamp from the image
        msg->header.frame_id = im_msg->header.frame_id;
        msg->header.stamp = im_msg->header.stamp;


        ROS_INFO("Image info Message filled");

        // Publish
        if (id == 1){
            camera_left_info_pub_.publish(msg);
        }
        else {
            camera_right_info_pub_.publish(msg);
        }

    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");
    CameraInfoPublisher ic;
    ROS_INFO("completed initialisation");
    ros::spin();
    return 0;
}

/*
 *
 *

        //sensor_msgs::CameraInfo msg;
int height = 200;
int width = 400;

msg.height = height;
msg.width = width;

//Distortion model
msg.distortion_model = "plumb_bob";

for (int i = 0; i < 5; i++)
{
    msg.D.push_back(0.0);
}

// Camera Intrinsics
msg.K[0] = 400.13;
msg.K[2] = 200.5;
msg.K[4] = 400.92;
msg.K[5] = 100.07;
msg.K[8] = 1.0;

//Rectification matrix
msg.R[0] = 1.0;
msg.R[4] = 1.0;
msg.R[8] = 1.0; // Identity matrix

//Projection matrix
msg.P[0] = 400.13;
msg.P[2] = 200.5;
msg.P[5] = 400.92;
msg.P[6] = 100.07;
msg.P[7] = 0.0;
msg.P[10] = 1.0;

if (id == 1){
    // Left camera - no offset
    msg.P[3] = 0.0;
}
else {
    // Right Camera - offset
    msg.P[3] = 2000.0;
}


// Binning
// msg.binning_x = 0;
// msg.binning_y = 0;

// Region of Interest
msg.roi.width = width;
msg.roi.height = height;
*/
