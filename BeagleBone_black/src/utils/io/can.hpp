/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 14. March 2018
 * Description:
 * CAN abstracts CANBUS networking. The type implements a Singleton design pattern.
 * CAN_FD is not supported.
 *
 * To the rest of the system CAN messages are described as can::Frame structure.
 * Sending messages is performed directly in the caller's thread.
 * Receiving messages is performed using a dedicated thread. This thread awaits
 * incoming messages and demultiplexes them to matching registered BMS/Motors units.
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

#include <cstdint>
#include <map>

#include "utils/concurrent/lock.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/utils.hpp"

namespace hyped {

// Forward declaration
namespace sensors { class BMS; }
namespace motor_control { class Controller; }

namespace utils {
namespace io {

// Import
using sensors::BMS;
using motor_control::Controller;

namespace can {

struct Frame {
  static constexpr uint32_t kExtendedMask = 0x80000000U;
  uint32_t  id;
  bool      extended;
  uint8_t   len;
  uint8_t   data[8];
};

}   // namespace can

class CanProccesor {
 public:
 /**
  * @brief To be called by CAN receive side. Object processes received CAN
  * message and updates its local data
  *
  * @param message received CAN message to be processed
  */
  virtual void processNewData(can::Frame& message) = 0;
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

  NO_COPY_ASSIGN(Can);
  // explicit Can(Can const&)    = delete;
  // void operator=(Can const&)  = delete;

  /**
   * @param  frame data to be sent
   * @return 1     iff data sent successfully
   */
  int send(const can::Frame& frame);

  /**
   * @brief BMS is registered for receiving CAN messages
   * @param bms pointer to BMS object to be registered
   */
  void registerBMS(BMS* bms);
  /**
   * @brief Controller is registered for receiving CAN messages
   * @param controller pointer to Controller object to be registered
   */
  void registerController(Controller* controller);

 private:
  /**
   * @param  frame output pointer to data to be filled
   * @return 1     iff data received successfully
   */
  int receive(can::Frame* frame);

  /**
   * @brief Process received message. Check whom does it belong to.
   * Send message to owner for processing.
   *
   * @param frame received CAN message
   */
  void processNewData(can::Frame* frame);

  /**
   * Blocking read and demultiplex messages based on configure id spaces
   */
  void run() override;

  Can();
  ~Can();

 private:
  int   socket_;
  bool  running_;
  std::map<uint32_t, BMS*>  bms_map_;
  Controller  *controller_array_[4];
  uint8_t array_counter_ = 0;
  concurrent::Lock          socket_lock_;
};

}}}   // namespace hyped::utils::io

#endif  // BEAGLEBONE_BLACK_UTILS_IO_CAN_HPP_
