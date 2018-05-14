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
#include "data/data.hpp"

namespace hyped {

using data::NavigationType;
using utils::concurrent::Thread;
using utils::Logger;

namespace motor_control {

class Main: public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  void run() override;

 private:
  void setupMotors();
  void accelerateMotors();
  void decelerateMotors();
  void stopMotors();
  int32_t accelerationVelocity(NavigationType velocity);
  int32_t decelerationVelocity(NavigationType velocity);
  int16_t accelerationTorque(NavigationType velocity);
  int16_t decelerationTorque(NavigationType velocity);
  data::Data& data_;
  data::StateMachine state_;
  Communicator communicator_;
  int32_t target_velocity_;
  int16_t target_torque_;
  bool motors_set_up_;
  bool motor_failure_;
  bool run_;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_MOTOR_CONTROL_MAIN_HPP_
