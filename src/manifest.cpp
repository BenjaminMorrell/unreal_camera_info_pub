#include <pluginlib/class_list_macros.h>
#include <unreal_camera_info_pub/resized_publisher.h>
#include <unreal_camera_info_pub/resized_subscriber.h>

PLUGINLIB_EXPORT_CLASS(ResizedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(ResizedSubscriber, image_transport::SubscriberPlugin)
