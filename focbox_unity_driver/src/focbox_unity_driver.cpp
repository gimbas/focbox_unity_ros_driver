// -*- mode:c++; fill-column: 100; -*-

#include "focbox_unity_driver/focbox_unity_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <focbox_unity_msgs/FocboxUnityCmd.h>
#include <focbox_unity_msgs/FocboxUnityStateStamped.h>

namespace focbox_unity_driver
{

FocboxUnityDriver::FocboxUnityDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  focbox_(std::string(),
        boost::bind(&FocboxUnityDriver::focboxUnityPacketCB, this, _1),
        boost::bind(&FocboxUnityDriver::focboxUnityErrorCB, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get focbox serial port address
  std::string port;
  if (!private_nh.getParam("port", port))
  {
    ROS_FATAL("FOCBOX communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try
  {
    focbox_.connect(port);
  }
  catch (SerialException e)
  {
    ROS_FATAL("Failed to connect to the FOCBOX, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create focbox state (telemetry) publisher
  state_pub_ = nh.advertise<focbox_unity_msgs::FocboxUnityStateStamped>("focbox_unity/state", 10);

  // subscribe to motor and servo command topics
  command_sub_ = nh.subscribe("focbox_unity/command", 10, &FocboxUnityDriver::commandCB, this);

  // create a 50Hz timer, used for state machine & pollingFOCBOX Unitytelemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &FocboxUnityDriver::timerCB, this);
}

void FocboxUnityDriver::timerCB(const ros::TimerEvent& event)
{
  //FOCBOX Unityinterface should not unexpectedly disconnect, but test for it anyway
  if(!focbox_.isConnected())
  {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for focbox version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // request version number, return packet will update the internal version numbers
    focbox_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      ROS_INFO("Connected to FOCBOX Unity with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING)
  {
    // poll for focbox state (telemetry)
    focbox_.requestState();
  }
  else
  {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void FocboxUnityDriver::focboxUnityPacketCB(const boost::shared_ptr<FocboxUnityPacket const>& packet)
{
  if(packet->name() == "Values")
  {
    boost::shared_ptr<FocboxUnityPacketValues const> values =
      boost::dynamic_pointer_cast<FocboxUnityPacketValues const>(packet);
// TODO:
    focbox_unity_msgs::FocboxUnityStateStamped::Ptr state_msg(new focbox_unity_msgs::FocboxUnityStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_fets1 = values->temp_fet1();
    state_msg->state.temperature_fets2 = values->temp_fet2();
    state_msg->state.motor1_temperature = values->temp_mot1();
    state_msg->state.motor2_temperature = values->temp_mot2();
    state_msg->state.motor1_current = values->motor_current1();
    state_msg->state.motor2_current = values->motor_current2();
    state_msg->state.current_input = values->current_in();
    state_msg->state.motor1_rpm = values->rpm1();
    state_msg->state.motor2_rpm = values->rpm2();
    state_msg->state.motor1_duty_cycle = values->duty_now1();
    state_msg->state.motor2_duty_cycle = values->duty_now2();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.motor1_displacement = values->tachometer1();
    state_msg->state.motor2_displacement = values->tachometer2();
    state_msg->state.motor1_distance_traveled = values->tachometer_abs1();
    state_msg->state.motor2_distance_traveled = values->tachometer_abs2();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion")
  {
    boost::shared_ptr<FocboxUnityPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<FocboxUnityPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void FocboxUnityDriver::focboxUnityErrorCB(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}


void FocboxUnityDriver::commandCB(const focbox_unity_msgs::FocboxUnityCmd::ConstPtr& command)
{
  //ROS_INFO("Got command: %d, %f, %f", command->command_type, command->motor1_parameter, command->motor2_parameter);

  if(driver_mode_ = MODE_OPERATING)
  {
    switch(command->command_type)
    {
      case 0: // COMM_SET_DUTY
      {
        double duty1 = duty_cycle_limit_.clip(command->motor1_parameter);
        double duty2 = duty_cycle_limit_.clip(command->motor2_parameter);
        focbox_.setDutyCycle(duty1, duty2);
      }
      break;

      case 1: // COMM_SET_CURRENT
      {
        double current1 = current_limit_.clip(command->motor1_parameter);
        double current2 = current_limit_.clip(command->motor2_parameter);
        focbox_.setCurrent(current1, current2);
      }
      break;

      case 2: // COMM_SET_CURRENT_BRAKE
      {
        double brake1 = brake_limit_.clip(command->motor1_parameter);
        double brake2 = brake_limit_.clip(command->motor2_parameter);
        focbox_.setBrake(brake1, brake2);
      }
      break;

      case 3: // COMM_SET_RPM
      {
        double rpm1 = speed_limit_.clip(command->motor1_parameter);
        double rpm2 = speed_limit_.clip(command->motor2_parameter);
        focbox_.setSpeed(rpm1, rpm2);
      }
      break;

      case 4: // COMM_SET_POSITION
      {
        double pos1 = position_limit_.clip(command->motor1_parameter);
        double pos2 = position_limit_.clip(command->motor2_parameter);
        focbox_.setPosition(pos1, pos2);
      }
      break;

      default:
      {
        ROS_INFO("Unknown focbox command");
      }
      break;
    }
  }
}

FocboxUnityDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min))
  {
    if (min_lower && param_min < *min_lower)
    {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper)
    {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      lower = param_min;
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max))
  {
    if (min_lower && param_max < *min_lower)
    {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper)
    {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      upper = param_max;
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper)
  {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double FocboxUnityDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower)
  {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper)
  {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace focbox_unity_driver
