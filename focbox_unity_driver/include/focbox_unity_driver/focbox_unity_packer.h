// -*- mode:c++; fill-column: 100; -*-

#ifndef FOCBOX_UNITY_PACKER_H_
#define FOCBOX_UNITY_PACKER_H_

#include <vector>
#include <map>
#include <string>

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include "focbox_unity_driver/v8stdint.h"
#include "focbox_unity_driver/focbox_unity_packet.h"

namespace focbox_unity_driver
{

/**
 * Class for creatingFOCBOX Unitypackets from raw data.
 */
class FocboxUnityPacker : private boost::noncopyable
{
public:
  /** Return the global factory object */
  static FocboxUnityPacker* getPacker();

  /**
   * Create a FocboxUnityPacket from a buffer (factory function). Packet must start (start of frame
   * character) at @p begin and complete (end of frame character) before *p end. The buffer element
   * at @p end is not examined, i.e. it can be the past-the-end element. Only returns a packet if
   * the packet is valid, i.e. valid size, matching checksum, complete etc. An empty pointer is
   * returned if a packet cannot be found or if it is invalid. If a valid packet is not found,
   * optional output parameter @what is set to a string providing a reason why a packet was not
   * found. If a packet was not found because additional bytes are needed on the buffer, optional
   * output parameter @p num_bytes_needed will contain the number of bytes needed to either
   * determine the size of the packet or complete the packet. Output parameters @p num_bytes_needed
   * and @p what will be set to 0 and empty if a valid packet is found.
   *
   * @param begin[in] Iterator to a buffer at the start-of-frame character
   * @param end[in] Iterator to the buffer past-the-end element.
   * @param num_bytes_needed[out] Number of bytes needed to determine the packet size or complete
   *                              the frame.
   * @param what[out] Human readable string giving a reason why the packet was not found.
   *
   * @return Pointer to a valid FocboxUnityPacket if successful. Otherwise, an empty pointer.
   */
  static FocboxUnityPacketPtr createPacket(const Buffer::const_iterator& begin,
                                    const Buffer::const_iterator& end,
                                    int* num_bytes_needed, std::string* what);

  typedef boost::function<FocboxUnityPacketPtr(boost::shared_ptr<FocboxUnityFrame>)> CreateFn;

  /** Register a packet type with the factory. */
  static void registerPacketType(int payload_id, CreateFn fn);

private:

  typedef std::map<int, CreateFn > PackerMap;
  static PackerMap* getMap();
};

/** Use this macro to register packets */
#define REGISTER_PACKET_TYPE(id, klass)   \
class klass##Packer \
{ \
public: \
  klass##Packer() \
  { \
    FocboxUnityPacker::registerPacketType((id), &klass##Packer::create); \
  } \
  static FocboxUnityPacketPtr create(boost::shared_ptr<FocboxUnityFrame> frame) \
  { \
    return FocboxUnityPacketPtr(new klass(frame)); \
  } \
}; \
static klass##Packer global_##klass##Packer;

} // namespace focbox_unity_driver

#endif // FOCBOX_UNITY_PACKER_H_
