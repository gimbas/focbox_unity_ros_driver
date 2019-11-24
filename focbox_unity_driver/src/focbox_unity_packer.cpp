// -*- mode:c++; fill-column: 100; -*-

#include "focbox_unity_driver/focbox_unity_packer.h"

#include <cassert>
#include <iterator>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "focbox_unity_driver/focbox_unity_packet.h"

namespace focbox_unity_driver
{

/** Construct map on first use */
FocboxUnityPacker::PackerMap* FocboxUnityPacker::getMap()
{
  static PackerMap m;
  return &m;
}

void FocboxUnityPacker::registerPacketType(int payload_id, CreateFn fn)
{
  PackerMap* p_map(getMap());
  assert(0 == p_map->count(payload_id));
  (*p_map)[payload_id] = fn;
}

/** Helper function for when createPacket can not create a packet */
FocboxUnityPacketPtr createFailed(int* p_num_bytes_needed, std::string* p_what,
                           const std::string& what, int num_bytes_needed = 0)
{
  if (p_num_bytes_needed != NULL) *p_num_bytes_needed = num_bytes_needed;
  if (p_what != NULL) *p_what = what;
  return FocboxUnityPacketPtr();
}

FocboxUnityPacketPtr FocboxUnityPacker::createPacket(const Buffer::const_iterator& begin,
                                              const Buffer::const_iterator& end,
                                              int* num_bytes_needed, std::string* what)
{
  // initialize output variables
  if (num_bytes_needed != NULL) *num_bytes_needed = 0;
  if (what != NULL) what->clear();

  // need at least FOCBOX_UNITY_MIN_FRAME_SIZE bytes in buffer
  int buffer_size(std::distance(begin, end));
  if (buffer_size < FocboxUnityFrame::FOCBOX_UNITY_MIN_FRAME_SIZE)
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame",
                        FocboxUnityFrame::FOCBOX_UNITY_MIN_FRAME_SIZE - buffer_size);

  // buffer must begin with a start-of-frame
  if (FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_SMALL_FRAME != *begin &&
      FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_LARGE_FRAME != *begin)
    return createFailed(num_bytes_needed, what, "Buffer must begin with start-of-frame character");

  // get a view of the payload
  BufferRangeConst view_payload;
  if (FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_SMALL_FRAME == *begin)
  {
    // payload size field is one byte
    view_payload.first = begin + 2;
    view_payload.second = view_payload.first + *(begin + 1);
  }
  else
  {
    assert(FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_LARGE_FRAME == *begin);
    // payload size field is two bytes
    view_payload.first = begin + 3;
    view_payload.second = view_payload.first + (*(begin + 1) << 8) + *(begin + 2);
  }

  // check length
  if(boost::distance(view_payload) > FocboxUnityFrame::FOCBOX_UNITY_MAX_PAYLOAD_SIZE)
    return createFailed(num_bytes_needed, what, "Invalid payload length");

  // get iterators to crc field, end-of-frame field, and a view of the whole frame
  Buffer::const_iterator iter_crc(view_payload.second);
  Buffer::const_iterator iter_eof(iter_crc + 2);
  BufferRangeConst view_frame(begin, iter_eof + 1);

  // do we have enough data in the buffer to complete the frame?
  int frame_size = boost::distance(view_frame);
  if (buffer_size < frame_size)
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame",
                        frame_size - buffer_size);

  // is the end-of-frame character valid?
  if (FocboxUnityFrame::FOCBOX_UNITY_EOF_VAL != *iter_eof)
    return createFailed(num_bytes_needed, what, "Invalid end-of-frame character");

  // is the crc valid?
  unsigned short crc = (static_cast<unsigned short>(*iter_crc) << 8) + *(iter_crc + 1);
  FocboxUnityFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*view_payload.first), boost::distance(view_payload));
  if (crc != crc_calc.checksum())
    return createFailed(num_bytes_needed, what, "Invalid checksum");

  // frame looks good, construct the raw frame
  boost::shared_ptr<FocboxUnityFrame> raw_frame(new FocboxUnityFrame(view_frame, view_payload));

  // if the packet has a payload, construct the corresponding subclass
  if (boost::distance(view_payload) > 0)
  {
    // get constructor function from payload id
    PackerMap* p_map(getMap());
    PackerMap::const_iterator search(p_map->find(*view_payload.first));
    if (search != p_map->end()) {
      return search->second(raw_frame);
    }
    else {
      // no subclass constructor for this packet
      return createFailed(num_bytes_needed, what, "Unkown payload type.");
    }

  }
  else {
    // no payload
    return createFailed(num_bytes_needed, what, "Frame does not have a payload");
  }
}


} // namesapce focbox_unity_driver
