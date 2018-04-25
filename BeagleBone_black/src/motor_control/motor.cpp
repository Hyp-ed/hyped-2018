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

#include "motor_control/motor.hpp"
#include <cstdint>

namespace hyped {
namespace motor_control {

Motor::Motor(Logger& log): log_(log)
{
  log_.INFO("MOTOR", "Motors initialised");
}

/**
  *  @brief { Function will return the readings of the 4 motor's RPM from the sensors }
  */
MotorsRpm Motor::getSpeed()
{
  // Read actual motor RPM over CAN. Set dummy values for now
  motors_rpm_.rpm_FR_ = rpm_;
  motors_rpm_.rpm_FL_ = rpm_;
  motors_rpm_.rpm_BL_ = rpm_;
  motors_rpm_.rpm_BR_ = rpm_;

  log_.DBG2("MOTOR", "Actual RPMs: FL %d, FR %d, BL %d, BR %d"
    , motors_rpm_.rpm_FL_
    , motors_rpm_.rpm_FR_
    , motors_rpm_.rpm_BL_
    , motors_rpm_.rpm_BR_);

  return motors_rpm_;
}

/**
  *  @brief  { Function will send RPM over CAN network to motor controllers }
  */
void Motor::setSpeed(int32_t rpm)
{
  this->rpm_ = rpm;
  log_.INFO("MOTOR", "Requested RPM %d", rpm);
}

/**
  *  @brief  { Check status of motors e.g. temperature and synchronisation.
  *            If critical failure, return True, otherwise return False }
  */
bool Motor::checkStatus()
{
  return false;
}

}}  // namespace hyped::motor_control
