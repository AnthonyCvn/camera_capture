#include "camera_capture/CameraCapture.hpp"
#include "camera_capture/SetCamera.h"

#include <string>
#include <cstdint>
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace camera_capture {

CameraCapture::CameraCapture(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      info_mgr_(nodeHandle)
{

//  info_pub_(nodeHandle),
//  info_mgr_(nodeHandle)

  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Set camera settings
  camSettings_.fps    = (int)fps_;
  camSettings_.height = (int)height_;
  camSettings_.width  = (int)width_;
  algorithm_.setCamSettings(camSettings_);

  // Launch the ImageTransport publisher
  image_transport::ImageTransport it(nodeHandle);

  if(isStereo_){
    imageLeftPublisher_  = it.advertise("stereo/left/image", 1);
    imageRightPublisher_ = it.advertise("stereo/right/image", 1);
  }else {
    imagePublisher_  = it.advertise("camera/image", 1);
  }

  // Launch the ROS service.
  serviceServer_ = nodeHandle_.advertiseService("set_camera",
                                                &CameraCapture::serviceCallback, this);

  // Open and configure the camera.
  cv::VideoCapture capture(cameraNum_);
  capture_ = capture;
  capture_.set(CV_CAP_PROP_FRAME_WIDTH, width_);
  capture_.set(CV_CAP_PROP_FRAME_HEIGHT, height_);

  if(!capture_.isOpened()){
	  ROS_ERROR("Could not open the camera.");
	  ros::requestShutdown();
  }

  ROS_INFO("Camera successfully launched.");

  // Camera info management and publisher.
  info_mgr_.setCameraName("camera"+std::to_string(cameraNum_));
  info_mgr_.loadCameraInfo(cameraInfoUrl_);
  info_pub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("camera_info", 1, this);

  // Launch the ROS timer interruption.
  timer_ = nodeHandle_.createTimer(ros::Duration(1/(double)fps_),
		  	  	  	  	  	  	   &CameraCapture::timerCallback, this);

  // Launch the ROS timer interruption.
  chrono_ = nodeHandle_.createTimer(ros::Duration(1/((double)fps_)),
                               &CameraCapture::chronoCallback, this);

  // Set the algorihm to running mode.
  algorithm_.setCamIsRunning(isRunning_);
}

CameraCapture::~CameraCapture()
{
}

bool CameraCapture::readParameters()
{
  if (nodeHandle_.getParam("camera_id", cameraId_)   &&
    nodeHandle_.getParam("is_running", isRunning_) &&
	  nodeHandle_.getParam("camera_num", cameraNum_) &&
	  nodeHandle_.getParam("is_stereo", isStereo_)   &&
    nodeHandle_.getParam("calib_file_path", cameraInfoUrl_)   &&
	  nodeHandle_.getParam("height", height_)     	 &&
	  nodeHandle_.getParam("width", width_)          &&
	  nodeHandle_.getParam("fps", fps_) ) return true;
  return false;
}

void CameraCapture::timerCallback(const ros::TimerEvent& event)
{
	static cv::Mat frame, left_image, right_image;

	ros::Time cap_time = ros::Time::now();
	// ROS_INFO("%i s %i ns \n\n", cap_time.sec, cap_time.nsec);

	if(algorithm_.getCamIsToConf())
	{
	  timer_.stop();
	  capture_.set(CV_CAP_PROP_FRAME_WIDTH, (int)algorithm_.getCamSettings().width);
	  capture_.set(CV_CAP_PROP_FRAME_HEIGHT,(int)algorithm_.getCamSettings().height);
	  timer_.setPeriod(ros::Duration(1.0/(double)algorithm_.getCamSettings().fps));
	  timer_.start();
	  ROS_INFO("Node successfully launched and camera %d configured.", cameraNum_);

	}
	if(algorithm_.getCamIsRunning())
	{
	  // Get a new frame from the camera
	  capture_ >> frame;

	  if(isStereo_){
	    // Extract left and right images from side-by-side.
      left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
      right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
      // OpenCv bridge to ROS standard message.
      sensor_msgs::ImagePtr msgim_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_image).toImageMsg();
      sensor_msgs::ImagePtr msgim_left  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_image ).toImageMsg();

      // Add header information
      msgim_right->header.stamp = cap_time ;
      msgim_right->header.frame_id = cameraId_;
      msgim_left->header.stamp = cap_time ;
      msgim_left->header.frame_id = cameraId_;

      // Publish images
      imageLeftPublisher_.publish(msgim_right);
      imageRightPublisher_.publish(msgim_left);
	  }else {
      // OpenCv bridge to ROS standard message.
      sensor_msgs::ImagePtr msgim = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      // Add header information
      msgim->header.stamp = cap_time ;
      msgim->header.frame_id = cameraId_;
      // Publish images
      imagePublisher_.publish(msgim);
	  }
	}// Camera will be deinitialized automatically in VideoCapture destructor


  sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(info_mgr_.getCameraInfo()));

  /*
  // Throw out not calibrated camera
  if (info->K[0] != 0.0 && (width_ != info->width || height_ != info->height)) {
    info.reset(new sensor_msgs::CameraInfo());
  }
*/
  // If we don't have a calibration, set the image dimensions
  if (info->K[0] == 0.0) {
    info->width = width_;
    info->height = height_;
  }

  info->height = height_;
  if(isStereo_)
    info->width = width_/2;
  else
    info->width = width_;

  info->header.stamp = cap_time;
  info->header.frame_id = cameraId_;

  info_pub_.publish(info);
}

void CameraCapture::chronoCallback(const ros::TimerEvent& event)
{
  //ros::Time cap_time = ros::Time::now();
  //ROS_INFO("%i s %i ns \n\n", cap_time.sec, cap_time.nsec);
}

bool CameraCapture::serviceCallback(camera_capture::SetCamera::Request &request,
                                    camera_capture::SetCamera::Response &response)
{
  response.success = true;

  // Set the user information to the algorithm.
  algorithm_.setCamIsToConf(request.is_toconf);
  algorithm_.setCamIsRunning(request.is_running);

  // Configure the camera.
  if(request.is_toconf)
  {
    camSettings_.fps    = (int)request.fps;
    camSettings_.height = (int)request.height;
    camSettings_.width  = (int)request.width;
    algorithm_.setCamSettings(camSettings_);
  }

  return true;
}

} /* namespace */
