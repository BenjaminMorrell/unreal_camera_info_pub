#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

class CameraInfoPublisher
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher camera_info_pub_; 

public:
    CameraInfoPublisher()
        : it_(nh_),
        image_sub_(it_.subscribe("/camera/left/image_raw",1, &CameraInfoPublisher::imageCb, this))
    {
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info",1000);
    }

    ~CameraInfoPublisher()
    {}

    
    void imageCb(const sensor_msgs::ImageConstPtr& im_msg)
    {
        ROS_INFO("In image callback for CameraInfoPublisher");
        // Create camera info message - load from cinfo
        sensor_msgs::CameraInfo msg;
        
        // Set frame id and time stamp from the image
        msg.header.frame_id = im_msg->header.frame_id;
        msg.header.stamp = im_msg->header.stamp;

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
        msg.P[3] = 2000.0;
        msg.P[5] = 400.92;
        msg.P[6] = 100.07;
        msg.P[7] = 0.0;
        msg.P[10] = 1.0;

        // Binning 
        // msg.binning_x = 0;
        // msg.binning_y = 0;

        // Region of Interest
        msg.roi.width = width;
        msg.roi.height = height;

        ROS_INFO("Image info Message filled");


        camera_info_pub_.publish(msg);

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
