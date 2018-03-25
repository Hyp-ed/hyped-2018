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

#ifndef BEAGLEBONE_BLACK_UTILS_IO_CAN_HPP_
#define BEAGLEBONE_BLACK_UTILS_IO_CAN_HPP_

#include <stdint.h>

#include <queue>

#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"


namespace hyped {
namespace utils {
namespace io {


struct CanFrame {
  uint32_t  id;
  uint8_t   len;
  uint8_t   data[8];
};


/**
 * Can implements singleton pattern to encapsulate one can interface, namely can0.
 * During object construction, can intereface is mapped onto socket_ member variable.
 * Furthermore, constructor spawns reading thread which waits on incoming can messages.
 * These messages are put into one of consuming queues based on configured id spaces.
 * The reading itself is performed in overriden run() method.
 */
class Can : public concurrent::Thread {
 public:
  static Can& getInstance()
  {
    static Can can;
    return can;
  }

  explicit Can(Can const&)    = delete;
  void operator=(Can const&)  = delete;

  /**
   * @param  frame data to be sent
   * @return 1     iff data sent successfully
   */
  int send(const CanFrame& frame);

  /**
   * @param  frame output pointer to data to be filled
   * @return 1     iff data received successfully
   */
  int receive(CanFrame* frame);

  /**
   * Perform thread-safe reading from BMS can buffer
   */
  CanFrame GetBMS();

  /**
   * Perform thread-safe reading from Proxi can buffer
   */
  CanFrame GetProxi();


 private:
  /**
   * Blocking read and demultiplex messages based on configure id spaces
   */
  void run() override;

  Can();
  ~Can();

 private:
  int socket_;
  int reading;

  concurrent::Lock bms_lock_;
  concurrent::Lock proxi_lock_;
  std::queue<CanFrame> bms_queue_;
  std::queue<CanFrame> proxi_queue_;
};

}}}   // namespace hyped::utils::io

#endif    // BEAGLEBONE_BLACK_UTILS_IO_CAN_HPP_
