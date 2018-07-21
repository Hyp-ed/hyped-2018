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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstdio>
#include "utils/system.hpp"

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

#define BAUD 115200
namespace hyped {
namespace utils {
namespace io {

Can::Can()
    : concurrent::Thread(0),
      uart_(System::getSystem().uart)
{
  if (!uart_) {
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
  } else {
    temp_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    // fd_ = stdin;
    int ret;
    struct termios tty = {};

    cfsetospeed(&tty, (speed_t)B115200);
    cfsetispeed(&tty, (speed_t)B115200);
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    cfmakeraw(&tty);
    tcflush(temp_, TCIFLUSH);
    ret = tcsetattr(temp_, TCSANOW, &tty);
    if (ret == -1)
    {
      perror("tcsetattr: ");
      exit(1);
    }
    printf("set attrs\n");
    fd_ = fdopen(temp_, "w+");
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

uint8_t charToVal(char c)
{
  if (c>='0' && c <= '9') return c - '0';
  else                    return (c-'A') + 10;
}

int Can::send(const can::Frame& frame)
{
  if (!uart_) {
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
  } else {
    if (fd_ == 0) {
      log_.ERR("Uart-CAN", "Unable to open /dev/ttyS1: ");
    }
    log_.DBG2("Uart-CAN", "opened port");

    log_.DBG2("CAN", "trying to send something");
    // checks, id <= ID_MAX, len <= LEN_MAX
    if (frame.len > 8) {
      log_.ERR("CAN", "trying to send message of more than 8 bytes, bytes: %d", frame.len);
      return 0;
    }
    {
      concurrent::ScopedLock L(&socket_lock_);
      if (frame.extended) {
        fprintf(fd_, "E%X %X %X %X %X %X %X %X %X\n", frame.id, frame.data[0], frame.data[1],
        frame.data[2], frame.data[3], frame.data[4],frame.data[5], frame.data[6],
        frame.data[7]);
      } else {
        fprintf(fd_, "S%X %X %X %X %X %X %X %X %X\n", frame.id, frame.data[0], frame.data[1],
        frame.data[2], frame.data[3], frame.data[4],frame.data[5], frame.data[6],
        frame.data[7]);
      }
    }


    log_.DBG1("CAN", "message with id %d sent, extended:%d",
        frame.id, frame.extended);
    return 1;
  }
}

void Can::run()
{
  /* these settings are static and can be held out of the hot path */
  can::Frame data;

  log_.INFO("CAN", "starting continuous reading");
  while (running_ && socket_ >= 0) {
    if (receive(&data)) {
      processNewData(&data);
      sleep(20);
    }
  }
  log_.INFO("CAN", "stopped continuous reading");

  if (socket_ >= 0) close(socket_);
}

int Can::receive(can::Frame* frame)
{
  size_t nBytes;
  can_frame raw_data;
    if (!uart_) {
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
    } else {
      //   // open the port
      if (fd_ == 0) {
        log_.ERR("Uart-CAN", "Unable to open /dev/ttyS1: ");
      }
      // log_.DBG2("Uart-CAN", "opened port");

      // char c;
      // uint8_t data[8];
      // int ret = fscanf(fd_, "%c%X %X %X %X %X %X %X %X %X", &c, 
      //       &frame->id, &frame->data[0], &frame->data[1], &frame->data[2],
      //       &frame->data[3], &frame->data[4], &frame->data[5], &frame->data[6],
      //       &frame->data[7]);
      // if (ret < 9) return 0;
      // frame->len      = 8;

      // log_.DBG1("CAN", "received %u, extended %d",
      //   frame->id, frame->extended);

      // return 1;
      char c;
      uint32_t id = 0x50;
      uint8_t data[8];
      int extended = 0;
      int id_set = 0;
      int i = 0;
      int ret = 0;
      while(1) {
        {
          // concurrent::ScopedLock L(&socket_lock_);
          // c = fgetc(fd_);
          int ret = read(temp_, &c, 1);
          if (!ret) continue;
        }
        if (c == 'S') {
            //printf("starting standard\r\n");
            extended = 0;
            i = 0;
            id = 0;
            id_set = 0;
        } else if (c == 'E') {
            extended = 1;
            i = 0;
            id = 0;
            id_set = 0;
        } else {
            if (id_set) {  // load data
                
                if (c == ' ' || c == '\r' || c == '\n') {
                    i++;
                    if (i == 8) { // data loaded
                        frame->id = id;
                        for (int i = 0; i < 8; i++)
                          frame->data[i] = data[i];
                        return 1;
                    } else {
                        data[i] = 0;
                    }
                } else {
                    data[i] <<= 4;
                    data[i] |= charToVal(c);
                
                }
            } else {
                if (c == ' ') {
                    id_set = 1;
                    data[i] = 0;
                    //printf("id set to %x", id);
                } else {
                    uint8_t val = charToVal(c);
                    id <<= 4;
                    id |= charToVal(c);
                    //printf("added %x to id %x", val, id);
                }
            }
        }
      }
      }
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
