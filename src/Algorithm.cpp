#include "camera_capture/Algorithm.hpp"

namespace camera_capture {

Algorithm::Algorithm()
    : istoconf_(true),
	  isrunning_(true)
{
}

Algorithm::~Algorithm()
{
}

void Algorithm::setCamSettings(const CamSettings cam_settings){
  camsettings_ = cam_settings;
}

CamSettings Algorithm::getCamSettings(){
  return camsettings_;
}

void Algorithm::setCamIsToConf(const bool conf_camera){
  istoconf_ = conf_camera;
}

bool Algorithm::getCamIsToConf()
{
  if(istoconf_)
  {
    istoconf_ = false;
    return true;
  }
  return false;
}

bool Algorithm::getCamIsRunning()
{
  return isrunning_;
}

void Algorithm::setCamIsRunning(const bool run_camera)
{
	isrunning_ = run_camera;

}

} /* namespace */
