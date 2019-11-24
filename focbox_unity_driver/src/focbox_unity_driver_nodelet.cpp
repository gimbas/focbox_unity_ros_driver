#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "focbox_unity_driver/focbox_unity_driver.h"

namespace focbox_unity_driver
{

class FocboxDriverNodelet: public nodelet::Nodelet
{
public:

  FocboxDriverNodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<FocboxUnityDriver> focbox_unity_driver_;

}; // class FocboxDriverNodelet

void FocboxDriverNodelet::onInit()
{
  NODELET_DEBUG("Initializing FOCBOX Unity driver nodelet");
  focbox_unity_driver_.reset(new FocboxUnityDriver(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace focbox_unity_driver

PLUGINLIB_EXPORT_CLASS(focbox_unity_driver::FocboxDriverNodelet, nodelet::Nodelet);
