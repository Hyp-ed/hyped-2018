/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * Sends 'broadcast' CAN messages to all four controllers and requests data from individual
 * controllers.
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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_COMMUNICATOR_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_COMMUNICATOR_HPP_

#include <cstdint>

#include "motor_control/controller.hpp"
#include "data/data.hpp"

namespace hyped {
// Forward declarations
namespace utils { class Logger; }
namespace utils { namespace io { namespace can { struct Frame; } } }

namespace motor_control {

using utils::Logger;
using utils::io::Can;

struct MotorVelocity {
  int32_t velocity_1;
  int32_t velocity_2;
  int32_t velocity_3;
  int32_t velocity_4;
};

struct MotorTorque {
  int16_t torque_1;
  int16_t torque_2;
  int16_t torque_3;
  int16_t torque_4;
};

/* Can Frames composed as follows:
 * Bytes 0-1: 0-11:  COB-ID = Function + Node ID,
 *            12:    RTR bit,
 *            13-16: Packet length
 * Bytes 1-2: Object Dictionary Index
 * Byte  3:   Object Dictionary Sub-Index
 * Bytes 4-7: Data
 */

class Communicator {
 public:
  explicit Communicator(Logger& log);
  /**
    *   @brief  Initialises the motor controllers
    */
  void initControllers();
  /**
    *   @brief  Registers the motor controllers to the can
    */
  void registerControllers();
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
  MotorVelocity requestActualVelocity();
  /**
    *  @brief  { Read actual torque from each controller and return motor velocity struct }
    */
  MotorTorque requestActualTorque();
  /*
   *  @brief stops all
   */
  void quickStopAll();
  int checkFailure();

 private:
  Logger& log_;
  Controller controller1_;
  Controller controller2_;
  Controller controller3_;
  Controller controller4_;
  MotorVelocity motor_velocity_;
  MotorTorque motor_torque_;
  int32_t target_velocity_;   // For testing only
  int32_t target_torque_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_COMMUNICATOR_HPP_
