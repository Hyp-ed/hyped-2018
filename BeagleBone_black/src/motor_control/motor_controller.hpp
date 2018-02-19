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

#ifndef BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_CONTROLLER_HPP_
#define BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_CONTROLLER_HPP_

#include "motor_control/motor.hpp"

namespace hyped {
namespace motor_control {

class MotorController {
 public:
  MotorController();
  void setupMotors();
  void accelerateMotors();
  void decelerateMotors();
  void stopMotors();
  int calculateAccelerationRPM(double translational_velocity);
  int calculateDecelerationRPM(double translational_velocity);

 private:
  Motor* motor;
  // these are temporary variables for testing only
  int rpm;
  int current_distance;
  int translational_velocity;
};

}}  // namespace hyped::motor_control

#endif  /* BEAGLEBONE_BLACK_MOTOR_CONTROL_MOTOR_CONTROLLER_HPP_ */
