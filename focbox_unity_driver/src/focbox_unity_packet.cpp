// -*- mode:c++; fill-column: 100; -*-

#include "focbox_unity_driver/focbox_unity_packet.h"

#include <cassert>
#include <iterator>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "focbox_unity_driver/focbox_unity_packer.h"
#include "focbox_unity_driver/datatypes.h"

namespace focbox_unity_driver
{

FocboxUnityFrame::FocboxUnityFrame(int payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256)
  {
    // single byte payload size
    frame_.reset(new Buffer(FOCBOX_UNITY_MIN_FRAME_SIZE + payload_size));
    *frame_->begin() = 2;
    *(frame_->begin() + 1) = payload_size;
    payload_.first = frame_->begin() + 2;
  }
  else
  {
    // two byte payload size
    frame_.reset(new Buffer(FOCBOX_UNITY_MIN_FRAME_SIZE + 1 + payload_size));
    *frame_->begin() = 3;
    *(frame_->begin() + 1) = payload_size >> 8;
    *(frame_->begin() + 2) = payload_size & 0xFF;
    payload_.first = frame_->begin() + 3;
  }

  payload_.second = payload_.first + payload_size;
  *(frame_->end() - 1) = 3;
}

FocboxUnityFrame::FocboxUnityFrame(const BufferRangeConst& frame, const BufferRangeConst& payload)
{
  /* FocboxUnityPacker::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
  assert(boost::distance(frame) >= FOCBOX_UNITY_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= FOCBOX_UNITY_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= FOCBOX_UNITY_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 &&
         std::distance(payload.second, frame.second) > 0);

  frame_.reset(new Buffer(boost::begin(frame), boost::end(frame)));
  payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
  payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
}

FocboxUnityPacket::FocboxUnityPacket(const std::string& name, int payload_size, int payload_id) :
  FocboxUnityFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(boost::distance(payload_) > 0);
  *payload_.first = payload_id;
}

FocboxUnityPacket::FocboxUnityPacket(const std::string& name, boost::shared_ptr<FocboxUnityFrame> raw) :
  FocboxUnityFrame(*raw), name_(name)
{
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketFWVersion::FocboxUnityPacketFWVersion(boost::shared_ptr<FocboxUnityFrame> raw) :
  FocboxUnityPacket("FWVersion", raw)
{
}

int FocboxUnityPacketFWVersion::fwMajor() const
{
  return *(payload_.first + 1);
}

int FocboxUnityPacketFWVersion::fwMinor() const
{
  return *(payload_.first + 2);
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, FocboxUnityPacketFWVersion)

FocboxUnityPacketRequestFWVersion::FocboxUnityPacketRequestFWVersion() :
  FocboxUnityPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketValues::FocboxUnityPacketValues(boost::shared_ptr<FocboxUnityFrame> raw) :
  FocboxUnityPacket("Values", raw)
{
}

double FocboxUnityPacketValues::temp_mos1() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 1)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 2)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_mos2() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 3)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 4)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_mos3() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 5)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 6)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_mos4() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 7)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 8)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_mos5() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 9)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 10)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_mos6() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 11)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 12)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::temp_pcb() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 13)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 14)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::current_motor() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 15)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 16)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 17)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 18)));
  return static_cast<double>(v) / 100.0;
}
double FocboxUnityPacketValues::current_in() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 19)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 20)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 21)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 22)));
  return static_cast<double>(v) / 100.0;
}
double FocboxUnityPacketValues::duty_now() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 23)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 24)));
  return static_cast<double>(v) / 1000.0;
}
double FocboxUnityPacketValues::rpm() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 25)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 26)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 27)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 28)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::v_in() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 29)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 30)));
  return static_cast<double>(v) / 10.0;
}
double FocboxUnityPacketValues::amp_hours() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 31)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 32)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 33)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 34)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::amp_hours_charged() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 35)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 36)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 37)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 38)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::watt_hours() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 39)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 40)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 41)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 42)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::watt_hours_charged() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 43)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 44)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 45)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 46)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::tachometer() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 47)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 48)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 49)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 50)));
  return static_cast<double>(v);
}
double FocboxUnityPacketValues::tachometer_abs() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 51)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 52)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 53)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 54)));
  return static_cast<double>(v);
}
int FocboxUnityPacketValues::fault_code() const
{
  return static_cast<int32_t>(*(payload_.first + 55));
}

REGISTER_PACKET_TYPE(COMM_GET_VALUES, FocboxUnityPacketValues)

FocboxUnityPacketRequestValues::FocboxUnityPacketRequestValues() :
  FocboxUnityPacket("RequestFWVersion", 1, COMM_GET_VALUES)
{
  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/


FocboxUnityPacketSetDuty::FocboxUnityPacketSetDuty(double duty) :
  FocboxUnityPacket("SetDuty", 5, COMM_SET_DUTY)
{
  /** @todo range check duty */

  int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketSetCurrent::FocboxUnityPacketSetCurrent(double current) :
  FocboxUnityPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketSetCurrentBrake::FocboxUnityPacketSetCurrentBrake(double current_brake) :
  FocboxUnityPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketSetRPM::FocboxUnityPacketSetRPM(double rpm) :
  FocboxUnityPacket("SetRPM", 5, COMM_SET_RPM)
{
  int32_t v = static_cast<int32_t>(rpm);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketSetPos::FocboxUnityPacketSetPos(double pos) :
  FocboxUnityPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

FocboxUnityPacketSetServoPos::FocboxUnityPacketSetServoPos(double servo_pos) :
  FocboxUnityPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  int16_t v = static_cast<int16_t>(servo_pos * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);

  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

} // namespace focbox_unity_driver
