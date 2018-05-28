/*
 * Author: Sean Mullan
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * A controller object represents one motor controller. Object is used to request data specific to
 * the controller.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_CONTROLLER_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_CONTROLLER_HPP_

#include <cstdint>
#include "utils/io/can.hpp"

namespace hyped {
// Forward declarations
namespace utils { class Logger; }
namespace utils { namespace io { class Can; } }
namespace utils { namespace io { class CanProccesor; } }
namespace utils { namespace io { namespace can { struct Frame; } } }

namespace motor_control {

using utils::Logger;
using utils::io::Can;
using utils::io::CanProccesor;

class Controller : public CanProccesor {
  friend Can;

 public:
  Controller(Logger& log, uint8_t id);
  /**
    *   @brief  Initialises the motor
    */
  void init();
  /**
    *   @brief Stops the motors as fast as possible
    */
  void quickStop();
  /**
    *  @brief  { Register controllers to receive messages on CAN bus }
    */
  void registerController();
  /**
    *  @brief  { Send 'broadcast' CAN message containing target velocity to all four
    *            controllers by setting Node-ID = 0 }
    */
  void sendTargetVelocity(int32_t target_velocity);
  /**
    *  @brief  { Send 'broadcast' CAN message containing target torque to all four
    *            controllers by setting Node-ID = 0 }
    */
  void sendTargetTorque(int16_t target_torque);
  /**
    *  @brief  { Read actual velocity from each controller and return motor velocity struct }
    */
  int32_t requestActualVelocity();
  /**
    *  @brief  { Read actual torque from each controller and return motor velocity struct }
    */
  int16_t requestActualTorque();
  /**
   * @brief To be called by CAN receive side. Controller processes received CAN
   * message and updates its local data
   *
   * @param message received CAN message to be processed
   */
  void processNewData(utils::io::can::Frame& message) override;
  int getFailure();
  int getId();


 private:
  Logger&  log_;
  Can&     can_;
  uint16_t node_id_;
  uint16_t sdo_receive;
  int failure_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_CONTROLLER_HPP_
