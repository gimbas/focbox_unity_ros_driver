// -*- mode:c++; fill-column: 100; -*-

#ifndef FOCBOX_UNITY_DRIVER_H_
#define FOCBOX_UNITY_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/optional.hpp>

#include "focbox_unity_driver/focbox_unity_interface.h"
#include "focbox_unity_driver/focbox_unity_packet.h"

namespace focbox_unity_driver
{

class FocboxUnityDriver
{
public:

  FocboxUnityDriver(ros::NodeHandle nh,
                    ros::NodeHandle private_nh);

private:
  // interface to the FOCBOX
  FocboxUnityInterface focbox_;
  void focboxUnityPacketCB(const boost::shared_ptr<FocboxUnityPacket const>& packet);
  void focboxUnityErrorCB(const std::string& error);

  // limits onFOCBOX Unitycommands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher servo_sensor_pub_;
  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;
  ros::Timer timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by focbox
  int fw_version_minor_;                ///< firmware minor version reported by focbox

  // ROS callbacks
  void timerCB(const ros::TimerEvent& event);
  void dutyCycleCB(const std_msgs::Float64::ConstPtr& duty_cycle);
  void currentCB(const std_msgs::Float64::ConstPtr& current);
  void brakeCB(const std_msgs::Float64::ConstPtr& brake);
  void speedCB(const std_msgs::Float64::ConstPtr& speed);
  void positionCB(const std_msgs::Float64::ConstPtr& position);
  void servoCB(const std_msgs::Float64::ConstPtr& servo);
};

} // namespace focbox_unity_driver

#endif // FOCBOX_UNITY_DRIVER_H_
