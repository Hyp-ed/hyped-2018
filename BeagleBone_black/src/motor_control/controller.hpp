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
    *  @brief  { Register controller to receive and transmit messages on CAN bus }
    */
  void registerController();
  /**
    *   @brief  { Applies configuration settings and sets controller to Operational }
    */
  void init();
  /**
    *  @brief  { Set target velocity in controller object dictionary }
    *
    *  @param[in] { Target velocity calculated in Main }
    */
  void sendTargetVelocity(int32_t target_velocity);
  /**
    *  @brief  { Set target torque in controller object dictionary }
    *
    *  @param[in] { Target torque calculated in Main }
    */
  void sendTargetTorque(int16_t target_torque);
  /**
    *  @brief  { Read actual velocity from controller }
    *
    *  @return { Actual velocity of motor }
    */
  int32_t requestActualVelocity();
  /**
    *  @brief  { Read actual torque from each controller }
    *
    *  @return { Actual torque of motor }
    */
  int16_t requestActualTorque();
  /**
   *  @brief { To be called by CAN receive side. Controller processes received CAN
   *          message and updates its local data }
   *
   *  @param[in] { CAN message to be processed }
   */
  void processNewData(utils::io::can::Frame& message) override;
  /**
    *  @brief { Sets controller into quickStop mode. Use in case of critical failure }
    */
  void quickStop();
  /*
   *  @brief { Return failure flag of controller }
   */
  bool getFailure();
  /*
   *  @brief { Return ID of controller }
   */
  uint8_t getId();

 private:
  Logger&  log_;
  Can&     can_;
  uint8_t node_id_;
  uint16_t sdo_receive;
  bool     failure_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_CONTROLLER_HPP_
