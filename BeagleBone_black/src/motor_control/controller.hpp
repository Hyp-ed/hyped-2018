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
#include "data/data.hpp"
#include "motor_control/controller_interface.hpp"

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

class Controller : public CanProccesor, public ControllerInterface {
  friend Can;

 public:
  Controller(Logger& log, uint8_t id);
  /**
   *   @brief { Does this CanProcessor own the corresponding can message? }
   *
   *   @return  { true - iff I am the owner of this can message }
   */
  bool hasId(uint32_t id, bool extended) override;
  /**
    *  @brief  { Register controller to receive and transmit messages on CAN bus }
    */
  void registerController() override;
  /**
    *   @brief  { Applies configuration settings }
    */
  void configure() override;
  /**
    *   @brief  { Checks for any warnings or errors, and enters operational state }
    */
  void enterOperational() override;
  /**
    *   @brief  { Enter preop state }
    */
  void enterPreOperational() override;
  /**
    *   @brief  { Checks controller statusword for state }
    */
  void checkState() override;
  /**
    *  @brief  { Set target velocity in controller object dictionary }
    *
    *  @param[in] { Target velocity calculated in Main }
    */
  void sendTargetVelocity(int32_t target_velocity) override;
  /**
    *  @brief  { Set target torque in controller object dictionary }
    *
    *  @param[in] { Target torque calculated in Main }
    */
  void sendTargetTorque(int16_t target_torque);
  /**
    *  @brief  { Send velocity sensor request to controller }
    */
  void updateActualVelocity() override;
  /**
    *  @brief  { Send torque sensor request to controller }
    */
  void updateActualTorque();
  /**
    *  @return { Actual velocity of motor }
    */
  int32_t getVelocity() override;
  /**
    *  @return { Actual torque of motor }
    */
  int16_t getTorque();
  /**
    *  @brief { Sets controller into quickStop mode. Use in case of critical failure }
    */
  void quickStop() override;
  /*
   *  @brief { Check error and warning register in controller }
   */
  void healthCheck() override;
  /*
   *  @brief { Return failure flag of controller }
   */
  bool getFailure() override;
  /*
   *  @brief { Return ID of controller }
   */
  uint8_t getId();
  /*
   * @brief { Returns state of controller }
   *
   * @return { ControllerState }
   */
  ControllerState getControllerState() override;
  /*
   *  @brief { To be called by CAN receive side. Controller processes received CAN
   *           message and updates its local data }
   *
   *  @param[in] { CAN message to be processed }
   */
  void processNewData(utils::io::can::Frame& message) override;
    /**
    *  @brief  { Send motor temperature sensor reading request }
    */
  void updateMotorTemp();
  /**
    *  @brief  { Send controller temperature sensor reading request }
    */
  void updateControllerTemp();
  /**
    *  @return { Actual temperature of motor }
    */
  uint8_t getMotorTemp();
  /**
    *  @return { Actual temperature of controller }
    */
  uint8_t getControllerTemp();

 private:
  /*
   * @brief { Sends a CAN frame but waits for a reply }
   */
  void sendSdoMessage(utils::io::can::Frame& message);
  /*
   * @brief { Set critical failure flag to true and write failure to data structure }
   */
  void throwCriticalFailure();
  /*
   * @brief { Sends state transition message to controller, leaving sufficient time for
   *          controller to change state. If state does not change, throw critical failure }
   *
   * @param[in] { CAN message to be sent, Controller state requested}
   */
  void requestStateTransition(utils::io::can::Frame& message, ControllerState state);
  /*
   *  @brief { Called by processNewData if Emergency message is detected. }
   *
   *  @param[in] { CAN message to be processed }
   */
  void processEmergencyMessage(utils::io::can::Frame& message);
  /*
   * @brief { Parses error message to find the problem }
   */
  void processErrorMessage(uint16_t error_message);
  /*
   *  @brief { Called by processNewData if SDO message is detected. }
   *
   *  @param[in] { CAN message to be processed }
   */
  void processSdoMessage(utils::io::can::Frame& message);
  /*
   *  @brief { Called by processNewData if NMT message is detected. }
   *
   *  @param[in] { CAN message to be processed }
   */
  void processNmtMessage(utils::io::can::Frame& message);

  Logger&      log_;
  Can&         can_;
  data::Data&  data_;
  data::Motors motor_data_;
  utils::io::can::Frame sdo_message_;
  utils::io::can::Frame nmt_message_;
  ControllerState state_;
  uint8_t  node_id_;
  bool     critical_failure_;
  int32_t  actual_velocity_;
  int16_t  actual_torque_;
  bool     sdo_frame_recieved_;
  uint8_t  motor_temperature_;
  uint8_t  controller_temperature_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_CONTROLLER_HPP_
