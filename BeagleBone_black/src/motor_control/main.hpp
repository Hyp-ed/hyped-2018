/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
 * Description:
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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_MAIN_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_MAIN_HPP_

#include <cstdint>

#include "motor_control/communicator.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/barrier.hpp"
#include "data/data.hpp"

namespace hyped {

using data::NavigationType;
using utils::concurrent::Thread;
using utils::concurrent::Barrier;
using utils::Logger;

namespace motor_control {

class Main: public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  /**
    *  @brief  { Runs motor control thread. Switches to correct motor state
    *            based on state machine state }
    */
  void run() override;

 private:
  /**
    *  @brief  { Establish CAN connections with motor controllers }
    */
  void initMotors();
  /**
    *  @brief  { Set motors into operational state }
    */
  void prepareMotors();
  /**
    *  @brief  { Enter controllers into pre operational state if config error occurs }
    */
  void accelerateMotors();
  /**
    *  @brief  { Will decelerate motors until total distance is reached }
    */
  void decelerateMotors();
  /**
    *  @brief  { Emergency stop, used in the case of a critical failure }
    */
  void stopMotors();
  /**
    *  @brief  { This function will run through slip ratio algorithm to calculate
    *            the desired acceleration velocity}
    *
    *  @param[in]  translational_velocity  { Value read from shared data structure }
    *
    *  @return  { Acceleration velocity calculation of type int }
    */
  int32_t accelerationVelocity(NavigationType velocity);
  /**
    *  @brief  { This function will run through slip ratio algorithm to calculate
    *            the desired deceleration velocity }
    *
    *  @param[in]  translational_velocity  { Value read from shared data structure }
    *
    *  @return  { Deceleration velocity calculation of type int }
    */
  int32_t decelerationVelocity(NavigationType velocity);
  /**
    *  @brief  { This function will calculate desired torque based on current
    *            translational velocity }
    *
    *  @param[in]  translational_velocity  { Value read from shared data structure }
    *
    *  @return  { 16 bit integer - target torque }
    */
  int16_t accelerationTorque(NavigationType velocity);
  /**
    *  @brief  { This function will calculate desired torque based on current
    *            translational velocity }
    *
    *  @param[in]  translational_velocity  { Value read from shared data structure }
    *
    *  @return  { 16 bit integer - target torque }
    */
  int16_t decelerationTorque(NavigationType velocity);
  /**
    *  @brief  { Continously listen for Go/Stop Comms commands to slowly move pod }
    */
  void servicePropulsion();
  /**
    *  @brief  { Updates the data structure with the velocity and torque of the motors }
    */
  void updateMotorData();
  /**
    *  @brief  { Updates the data structure motor failure has occured }
    */
  void updateMotorFailure();

  data::Data& data_;
  data::StateMachine state_;
  data::Motors motor_data_;
  Barrier post_calibration_barrier_;
  Communicator* communicator_;
  int32_t target_velocity_;
  int16_t target_torque_;
  bool run_;
  bool nav_calib_;
  bool motors_init_;
  bool motors_ready_;
  // bool motors_operational_;  // TODO(anyone) add back in if needed
  bool motor_failure_;
  bool all_motors_stopped_;
  MotorVelocity motor_velocity_;
  MotorTorque motor_torque_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_MAIN_HPP_
