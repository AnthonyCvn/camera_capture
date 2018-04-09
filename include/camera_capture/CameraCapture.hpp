#pragma once

#include "camera_capture/Algorithm.hpp"
#include "camera_capture/SetCamera.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>

namespace camera_capture {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class CameraCapture
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  CameraCapture(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~CameraCapture();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS timer callback.
   * @param TimerEvent the event structure of the timer.
   */
  void timerCallback(const ros::TimerEvent &event);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(camera_capture::SetCamera::Request  &request,
                       camera_capture::SetCamera::Response &response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Left image transport publisher (stereo mode).
  image_transport::Publisher imageLeftPublisher_;

  //! Right image transport publisher (stereo mode).
  image_transport::Publisher imageRightPublisher_;

  //! Image transport publisher.
  image_transport::Publisher imagePublisher_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! ROS timer.
  ros::Timer timer_;

  //! Camera info publisher.
  ros::Publisher info_pub_;

  //! Camera info manager.
  camera_info_manager::CameraInfoManager info_mgr_;

  //! Algorithm computation object.
  Algorithm algorithm_;

  //! Camera settings wrapper.
  CamSettings camSettings_;

  //! UVC camera capture.
  cv::VideoCapture capture_;

  //! ROS server parameters; run camera.
  bool isRunning_;

  //! ROS server parameters; camera number.
  int cameraNum_;

  //! ROS server parameters; for stereo mode.
  bool isStereo_;

  //! Camera info folder location.
  std::string cameraInfoUrl_;

  //! Camera frame ID.
  std::string cameraId_;

  //! ROS server parameters; height resolution.
  int height_;

  //! ROS server parameters; width resolution.
  int width_;

  //! ROS server parameters; frames per second.
  int fps_;

};

} /* namespace */
