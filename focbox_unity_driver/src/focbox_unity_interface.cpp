// -*- mode:c++; fill-column: 100; -*-

#include "focbox_unity_driver/focbox_unity_interface.h"

#include <pthread.h>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <serial/serial.h>
#include <boost/crc.hpp>

#include "focbox_unity_driver/focbox_unity_packer.h"

namespace focbox_unity_driver
{

class FocboxUnityInterface::Impl
{
public:
  Impl() :
    serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
            serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none)
  {}

  void* rxThread(void);

  static void* rxThreadHelper(void *context)
  {
    return ((FocboxUnityInterface::Impl*)context)->rxThread();
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  serial::Serial serial_;
  FocboxUnityFrame::CRC send_crc_;
};

void* FocboxUnityInterface::Impl::rxThread(void)
{
  Buffer buffer;
  buffer.reserve(4096);

  while(rx_thread_run_)
  {

    int bytes_needed = FocboxUnityFrame::FOCBOX_UNITY_MIN_FRAME_SIZE;
    if (!buffer.empty()) {

      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while(iter != buffer.end())
      {

        // check if valid start-of-frame character
        if (FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_SMALL_FRAME == *iter ||
            FocboxUnityFrame::FOCBOX_UNITY_SOF_VAL_LARGE_FRAME == *iter)
        {

          // good start, now attempt to create packet
          std::string error;
          FocboxUnityPacketConstPtr packet =
            FocboxUnityPacker::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet)
          {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0)
            {
              std::ostringstream ss;
              ss << "Out-of-sync with FOCBOX, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0)
          {
            // need more data, break out of while loop
            break; // for (iter_sof...
          }
          else
          {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end())
        bytes_needed = FocboxUnityFrame::FOCBOX_UNITY_MIN_FRAME_SIZE;

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with FOCBOX, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }

    // attempt to read at least bytes_needed bytes from the serial port
    int bytes_to_read =
      std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));
    int bytes_read = serial_.read(buffer, bytes_to_read);
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
      error_handler_("Possibly out-of-sync with FOCBOX, read timout in the middle of a frame.");
    }

  }
}


FocboxUnityInterface::FocboxUnityInterface(const std::string& port,
                             const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler) :
  impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty())
    connect(port);
}

FocboxUnityInterface::~FocboxUnityInterface()
{
  disconnect();
}

void FocboxUnityInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void FocboxUnityInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void FocboxUnityInterface::connect(const std::string& port)
{
  // todo - mutex?

  if (isConnected()) {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try
  {
    impl_->serial_.setPort(port);
    impl_->serial_.open();
  }
  catch(const std::exception& e) 
  {
      std::stringstream ss;
      ss << "Failed to open the serial port to the FOCBOX. " << e.what();
      throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->rx_thread_run_ = true;
  int result =
    pthread_create(&impl_->rx_thread_, NULL, &FocboxUnityInterface::Impl::rxThreadHelper, impl_.get());
  assert(0 == result);
}

void FocboxUnityInterface::disconnect()
{
  // todo - mutex?

  if (isConnected())
  {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    int result = pthread_join(impl_->rx_thread_, NULL);
    assert(0 == result);

    impl_->serial_.close();
  }
}

bool FocboxUnityInterface::isConnected() const
{
  return impl_->serial_.isOpen();
}

void FocboxUnityInterface::send(const FocboxUnityPacket& packet)
{
  size_t written = impl_->serial_.write(packet.frame());
  if(written != packet.frame().size())
  {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.frame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void FocboxUnityInterface::requestFWVersion()
{
  send(FocboxUnityPacketRequestFWVersion());
}

void FocboxUnityInterface::requestState()
{
  send(FocboxUnityPacketRequestValues());
}

void FocboxUnityInterface::setDutyCycle(double duty_cycle)
{
  send(FocboxUnityPacketSetDuty(duty_cycle));
}

void FocboxUnityInterface::setCurrent(double current)
{
  send(FocboxUnityPacketSetCurrent(current));
}

void FocboxUnityInterface::setBrake(double brake)
{
  send(FocboxUnityPacketSetCurrentBrake(brake));
}

void FocboxUnityInterface::setSpeed(double speed)
{
  send(FocboxUnityPacketSetRPM(speed));
}

void FocboxUnityInterface::setPosition(double position)
{
  send(FocboxUnityPacketSetPos(position));
}

void FocboxUnityInterface::setServo(double servo)
{
  send(FocboxUnityPacketSetServoPos(servo));
}

} // namespace focbox_unity_driver
