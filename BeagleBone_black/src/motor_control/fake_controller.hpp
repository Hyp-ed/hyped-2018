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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_FAKE_CONTROLLER_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_FAKE_CONTROLLER_HPP_

#include <cstdint>
#include <fstream>
#include "data/data.hpp"
#include "utils/timer.hpp"
#include "motor_control/controller_interface.hpp"
#include "data/data.hpp"

namespace hyped {
namespace utils { class Logger; }
namespace motor_control {

using utils::Logger;
using utils::Timer;

class FakeController : public ControllerInterface {
 public:

  FakeController(Logger& log, uint8_t id, bool faulty);
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
    *  @brief  { Send velocity sensor request to controller }
    */
  void updateActualVelocity() override;
  /**
    *  @return { Actual velocity of motor }
    */
  int32_t getVelocity() override;
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
   * @brief { Returns state of controller }
   *
   * @return { ControllerState }
   */
  ControllerState getControllerState() override;

 private:
  void startTimer();
  Logger&        log_;
  data::Data&    data_;
  data::Motors motor_data_;
  ControllerState state_;
  std::ofstream RPMvTime;
  Timer timer;
  uint8_t  node_id_;
  bool     critical_failure_;
  int32_t  actual_velocity_;
  bool     faulty_;
  uint64_t start_time_;
  bool     timer_started_;
  uint64_t fail_time_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_FAKE_CONTROLLER_HPP_