#include <ros/ros.h>
#include "camera_capture/CameraCapture.hpp"

#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_capture");
  ros::NodeHandle nodeHandle("~");

  camera_capture::CameraCapture CameraCapture(nodeHandle);

  ros::spin();

  return 0;
}
