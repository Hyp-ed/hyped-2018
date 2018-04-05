/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_HPP_

#include <cstdint>
#include "utils/logger.hpp"

namespace hyped {

using utils::Logger;

namespace motor_control {
// Contains the RPM of each of the motors
struct MotorsRpm {
  int32_t rpm_FL;
  int32_t rpm_FR;
  int32_t rpm_BL;
  int32_t rpm_BR;
};

class Motor {
 public:
  explicit Motor(Logger& log);
  void setSpeed(int32_t rpm);
  MotorsRpm getSpeed();
  bool checkStatus();

 private:
  MotorsRpm motors_rpm_;
  int32_t rpm;   // For testing only
  Logger& log_;
};

}}  // namespace hyped::motor_control

#endif  /* BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_HPP_ */
