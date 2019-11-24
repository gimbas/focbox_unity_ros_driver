// -*- mode:c++; fill-column: 100; -*-

#include "focbox_unity_driver/focbox_unity_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <focbox_unity_msgs/FocboxUnityStateStamped.h>

namespace focbox_unity_driver
{

FocboxUnityDriver::FocboxUnityDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  focbox_(std::string(),
        boost::bind(&FocboxUnityDriver::focboxUnityPacketCB, this, _1),
        boost::bind(&FocboxUnityDriver::focboxUnityErrorCB, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"), servo_limit_(private_nh, "servo", 0.0, 1.0),
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
  try {
    focbox_.connect(port);
  }
  catch (SerialException e)
  {
    ROS_FATAL("Failed to connect to the FOCBOX, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create focbox state (telemetry) publisher
  state_pub_ = nh.advertise<focbox_unity_msgs::FocboxUnityStateStamped>("sensors/core", 10);

  // since focbox state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10,
                                 &FocboxUnityDriver::dutyCycleCB, this);
  current_sub_ = nh.subscribe("commands/motor/current", 10, &FocboxUnityDriver::currentCB, this);
  brake_sub_ = nh.subscribe("commands/motor/brake", 10, &FocboxUnityDriver::brakeCB, this);
  speed_sub_ = nh.subscribe("commands/motor/speed", 10, &FocboxUnityDriver::speedCB, this);
  position_sub_ = nh.subscribe("commands/motor/position", 10, &FocboxUnityDriver::positionCB, this);
  servo_sub_ = nh.subscribe("commands/servo/position", 10, &FocboxUnityDriver::servoCB, this);

  // create a 50Hz timer, used for state machine & pollingFOCBOX Unitytelemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &FocboxUnityDriver::timerCB, this);
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the focbox interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict focbox bounds (from focbox config) and command detect bounds errors
  */

void FocboxUnityDriver::timerCB(const ros::TimerEvent& event)
{
  //FOCBOX Unityinterface should not unexpectedly disconnect, but test for it anyway
  if (!focbox_.isConnected())
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
      ROS_INFO("Connected toFOCBOX Unitywith firmware version %d.%d",
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
  if (packet->name() == "Values")
  {
    boost::shared_ptr<FocboxUnityPacketValues const> values =
      boost::dynamic_pointer_cast<FocboxUnityPacketValues const>(packet);

    focbox_unity_msgs::FocboxUnityStateStamped::Ptr state_msg(new focbox_unity_msgs::FocboxUnityStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
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

/**
 * @param duty_cycle CommandedFOCBOX Unityduty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that theFOCBOX Unitymay impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void FocboxUnityDriver::dutyCycleCB(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    focbox_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current CommandedFOCBOX Unitycurrent in Amps. Any value is accepted by this driver. However,
 *                note that theFOCBOX Unitymay impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void FocboxUnityDriver::currentCB(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    focbox_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake CommandedFOCBOX Unitybraking current in Amps. Any value is accepted by this driver.
 *              However, note that theFOCBOX Unitymay impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void FocboxUnityDriver::brakeCB(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    focbox_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed CommandedFOCBOX Unityspeed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that theFOCBOX Unitymay impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void FocboxUnityDriver::speedCB(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    focbox_.setSpeed(speed_limit_.clip(speed->data));
  }
}

/**
 * @param position CommandedFOCBOX Unitymotor position in radians. Any value is accepted by this driver.
 *                 Note that theFOCBOX Unitymust be in encoder mode for this command to have an effect.
 */
void FocboxUnityDriver::positionCB(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    // ROS uses radians butFOCBOX Unityseems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    focbox_.setPosition(position_deg);
  }
}

/**
 * @param servo CommandedFOCBOX Unityservo output position. Valid range is 0 to 1.
 */
void FocboxUnityDriver::servoCB(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    double servo_clipped(servo_limit_.clip(servo->data));
    focbox_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
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
