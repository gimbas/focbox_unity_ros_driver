// -*- mode:c++; fill-column: 100; -*-

#ifndef FOCBOX_UNITY_PACKET_H_
#define FOCBOX_UNITY_PACKET_H_

#include <string>
#include <vector>
#include <utility>

#include <boost/crc.hpp>
#include <boost/shared_ptr.hpp>

#include "focbox_unity_driver/v8stdint.h"

namespace focbox_unity_driver
{

typedef std::vector<uint8_t> Buffer;
typedef std::pair<Buffer::iterator, Buffer::iterator> BufferRange;
typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;

/** The raw frame for communicating with theFOCBOX Unity*/
class FocboxUnityFrame
{
public:
  virtual ~FocboxUnityFrame() {}

  // getters
  virtual const Buffer& frame() const {return *frame_;}

  //FOCBOX Unitypacket properties
  static const int FOCBOX_UNITY_MAX_PAYLOAD_SIZE = 1024;          ///< MaximumFOCBOX Unitypayload size, in bytes
  static const int FOCBOX_UNITY_MIN_FRAME_SIZE = 5;               ///< SmallestFOCBOX Unityframe size, in bytes
  static const int FOCBOX_UNITY_MAX_FRAME_SIZE = 6 + FOCBOX_UNITY_MAX_PAYLOAD_SIZE; ///< LargestFOCBOX Unityframe size, in bytes
  static const unsigned int FOCBOX_UNITY_SOF_VAL_SMALL_FRAME = 2; ///<FOCBOX Unitystart of "small" frame value
  static const unsigned int FOCBOX_UNITY_SOF_VAL_LARGE_FRAME = 3; ///<FOCBOX Unitystart of "large" frame value
  static const unsigned int FOCBOX_UNITY_EOF_VAL = 3;             ///<FOCBOX Unityend-of-frame value

  /** CRC parameters for theFOCBOX Unity*/
  typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

protected:
  /** Construct frame with specified payload size. */
  FocboxUnityFrame(int payload_size);

  boost::shared_ptr<Buffer> frame_; ///< Stores frame data, shared_ptr for shallow copy
  BufferRange payload_;             ///< View into frame's payload section

private:
  /** Construct from buffer. Used by FocboxUnityPacker factory. */
  FocboxUnityFrame(const BufferRangeConst& frame, const BufferRangeConst& payload);

  /** Give FocboxUnityPacker access to private constructor. */
  friend class FocboxUnityPacker;
};

/*------------------------------------------------------------------------------------------------*/

/** A FocboxUnityPacket is a FocboxUnityFrame with a non-zero length payload */
class FocboxUnityPacket : public FocboxUnityFrame
{
public:
  virtual ~FocboxUnityPacket() {}

  virtual const std::string& name() const {return name_;}

protected:
  FocboxUnityPacket(const std::string& name, int payload_size, int payload_id);
  FocboxUnityPacket(const std::string& name, boost::shared_ptr<FocboxUnityFrame> raw);

private:
  std::string name_;
};

typedef boost::shared_ptr<FocboxUnityPacket> FocboxUnityPacketPtr;
typedef boost::shared_ptr<FocboxUnityPacket const> FocboxUnityPacketConstPtr;

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketFWVersion : public FocboxUnityPacket
{
public:
  FocboxUnityPacketFWVersion(boost::shared_ptr<FocboxUnityFrame> raw);

  int fwMajor() const;
  int fwMinor() const;

};

class FocboxUnityPacketRequestFWVersion : public FocboxUnityPacket
{
public:
  FocboxUnityPacketRequestFWVersion();

};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketValues : public FocboxUnityPacket
{
public:
  FocboxUnityPacketValues(boost::shared_ptr<FocboxUnityFrame> raw);


double temp_fet1() const;
double temp_fet2() const;
double temp_mot1() const;
double temp_mot2() const;
double motor_current1() const;
double motor_current2() const;
double current_in() const;
double current_id1() const;
double current_id2() const;
double current_iq1() const;
double current_iq2() const;
double duty_now1() const;
double duty_now2() const;
double rpm1() const;
double rpm2() const;
double v_in() const;
double amp_hours() const;
double amp_hours_charged() const;
double watt_hours() const;
double watt_hours_charged() const;
double tachometer1() const;
double tachometer2() const;
double tachometer_abs1() const;
double tachometer_abs2() const;
int fault_code() const;

};

class FocboxUnityPacketRequestValues : public FocboxUnityPacket
{
public:
  FocboxUnityPacketRequestValues();
};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketSetDuty : public FocboxUnityPacket
{
public:
  FocboxUnityPacketSetDuty(double duty1, double duty2);
};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketSetCurrent : public FocboxUnityPacket
{
public:
  FocboxUnityPacketSetCurrent(double current1, double current2);
};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketSetCurrentBrake : public FocboxUnityPacket
{
public:
  FocboxUnityPacketSetCurrentBrake(double brake1, double brake2); //  double duty() const;
};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketSetRPM : public FocboxUnityPacket
{
public:
  FocboxUnityPacketSetRPM(double rpm1, double rpm2);
};

/*------------------------------------------------------------------------------------------------*/

class FocboxUnityPacketSetPos : public FocboxUnityPacket
{
public:
  FocboxUnityPacketSetPos(double pos1, double pos2);
};

} // namespace focbox_unity_driver

#endif // FOCBOX_UNITY_PACKET_H_
