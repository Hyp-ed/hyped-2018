/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 14. March 2018
 * Description:
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "utils/io/can.hpp"

#include <stdio.h>
#include <unistd.h>

#include <sys/socket.h>
#include <net/if.h>

#ifndef WIN
#include <linux/can.h>
#else
#define CAN_MAX_DLEN 8
struct can_frame {
  uint32_t can_id;    /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t    __pad;   /* padding */
  uint8_t    __res0;  /* reserved / padding */
  uint8_t    __res1;  /* reserved / padding */
  uint8_t    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

#ifndef PF_CAN
#define PF_CAN 29
#define AF_CAN PF_CAN
#endif

#define CAN_RAW 1
#define CAN_MTU (sizeof(struct can_frame))
struct sockaddr_can {
  uint16_t can_family;
  int      can_ifindex;
  union {
    struct { uint32_t rx_id, tx_id; } tp;
  } can_addr;
};

#endif   // CAN

namespace hyped {
namespace utils {
namespace io {

Can::Can()
    : concurrent::Thread(0)
{
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    log_.ERR("CAN", "Could not open can socket");
    return;
  }

  sockaddr_can addr;
  addr.can_family   = AF_CAN;
  addr.can_ifindex  = if_nametoindex("can0");   // ifr.ifr_ifindex;

  if (addr.can_ifindex == 0) {
    log_.ERR("CAN", "Could not find can0 network interface");
    close(socket_);
    socket_ = -1;
    return;
  }

  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    log_.ERR("CAN", "Could not bind can socket");
    close(socket_);
    socket_ = -1;
    return;
  }

  log_.INFO("CAN", "socket successfully created");
}

Can::~Can()
{
  running_ = false;
}

void Can::start()
{
  if (running_) return;   // already started

  running_ = true;
  concurrent::Thread::start();
}

int Can::send(const can::Frame& frame)
{
  if (socket_ < 0) return 0;  // early exit if no can device present

  can_frame can;
  log_.DBG2("CAN", "trying to send something");
  // checks, id <= ID_MAX, len <= LEN_MAX
  if (frame.len > 8) {
    log_.ERR("CAN", "trying to send message of more than 8 bytes, bytes: %d", frame.len);
    return 0;
  }

  can.can_id  = frame.id;
  can.can_id |= frame.extended ? can::Frame::kExtendedMask : 0;  // add extended id flag
  can.can_dlc = frame.len;
  for (int i = 0; i < frame.len; i++) {
    can.data[i] = frame.data[i];
  }

  {
    concurrent::ScopedLock L(&socket_lock_);
    if (write(socket_, &can, CAN_MTU) != CAN_MTU) {
      log_.ERR("CAN", "cannot write to socket");
      return 0;
    }
  }

  log_.DBG1("CAN", "message with id %d sent, extended:%d",
      frame.id, frame.extended);
  return 1;
}

void Can::run()
{
  /* these settings are static and can be held out of the hot path */
  can::Frame data;

  log_.INFO("CAN", "starting continuous reading");
  while (running_ && socket_ >= 0) {
    receive(&data);
    processNewData(&data);
  }
  log_.INFO("CAN", "stopped continuous reading");

  if (socket_ >= 0) close(socket_);
}

int Can::receive(can::Frame* frame)
{
  size_t nBytes;
  can_frame raw_data;

  nBytes = read(socket_, &raw_data, CAN_MTU);
  if (nBytes != CAN_MTU) {
    // perror("read");
    log_.ERR("CAN", "cannot read from socket");
    return 0;
  }

  frame->id       = raw_data.can_id & ~can::Frame::kExtendedMask;
  frame->extended = raw_data.can_id & can::Frame::kExtendedMask;
  frame->len      = raw_data.can_dlc;
  for (int i = 0; i < frame->len; i++) {
    frame->data[i] = raw_data.data[i];
  }
  log_.DBG1("CAN", "received %u %u, extended %d",
      raw_data.can_id, frame->id, frame->extended);
  return 1;
}

void Can::processNewData(can::Frame* message)
{
  CanProccesor* owner = 0;

  for (CanProccesor* processor : processors_) {
    if (processor->hasId(message->id, message->extended)) {
      owner = processor;
      break;
    }
  }

  if (owner) {
    owner->processNewData(*message);
  } else {
    log_.ERR("CAN", "did not find owner of received CAN message with id %d", message->id);
  }
}

void Can::registerProcessor(CanProccesor* processor)
{
  processors_.push_back(processor);
}

}}}   // namespace hyped::utils::io
