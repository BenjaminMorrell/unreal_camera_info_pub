#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

class CameraInfoPublisher
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher camera_info_pub_;  
    // camera_info_manager::CameraInfoManager cinfo_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;


public:
    CameraInfoPublisher()
        : it_(nh_),
        image_sub_(it_.subscribe("/camera/image",1, &CameraInfoPublisher::imageCb, this)),
        // image_pub_(it_.advertiseCamera("image_raw",1)),
        cinfo_(new camera_info_manager::CameraInfoManager(nh_))
    {
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info",1000);
    }

    ~CameraInfoPublisher()
    {}

    void setup(void){

        std::string url = "/home/bjm/Development/voxblox_ws/src/image_transport_tutorial/src/camera_left_info.yaml";

        if (cinfo_->validateURL(url)){
            cinfo_->loadCameraInfo(url);
            ROS_INFO("loaded url for camera properties");
        }
        else
        {
            ROS_INFO("url for config files not working");
        }


    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& im_msg)
    {
        ROS_INFO("In image callback for CameraInfoPublisher");
        // Create camera info message - load from cinfo
        sensor_msgs::CameraInfo ci = cinfo_->getCameraInfo();
        
        // Set frame id and time stamp from the image
        ci.header.frame_id = im_msg->header.frame_id;
        ci.header.stamp = im_msg->header.stamp;

        /*        
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
        msg.R[9] = 1.0; // Identity matrix
        
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

        //Send the message
        */

        camera_info_pub_.publish(ci);

    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");
    CameraInfoPublisher ic;
    ROS_INFO("completed initialisation");
    ic.setup();
    ros::spin();
    return 0;
}