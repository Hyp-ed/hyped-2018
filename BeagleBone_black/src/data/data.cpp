/*
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description:
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may
 *    not use this file except in compliance with the License. You may obtain a
 *    copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *    WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *    License for the specific language governing permissions and limitations
 *    under the License.
 */

#include "data.hpp"

namespace hyped {

// imports
using utils::concurrent::ScopedLock;

namespace data {

Data& Data::getInstance()
{
  static Data d;
  return d;
}

StateMachine Data::getStateMachineData()
{
  ScopedLock L(&lock_state_machine_);
  return state_machine_;
}

void Data::setStateMachineData(const StateMachine& sm_data)
{
  ScopedLock L(&lock_state_machine_);
  state_machine_ = sm_data;
}

Navigation Data::getNavigationData()
{
  ScopedLock L(&lock_navigation_);
  return navigation_;
}

void Data::setNavigationData(const Navigation& nav_data)
{
  ScopedLock L(&lock_navigation_);
  navigation_ = nav_data;
}

Sensors Data::getSensorsData()
{
  ScopedLock L(&lock_sensors_);
  return sensors_;
}

void Data::setSensorsData(const Sensors& sensors_data)
{
  ScopedLock L(&lock_sensors_);
  sensors_ = sensors_data;
}

Sensors Data::getBatteryData()
{
  ScopedLock L(&lock_sensors_);
  return sensors_;
}

void Data::setBatteryData(const Batteries& batteries_data)
{
  ScopedLock L(&lock_batteries_);
  batteries_ = batteries_data;
}

Motors Data::getMotorData()
{
  ScopedLock L(&lock_motors_);
  return motors_;
}

void Data::setMotorData(const Motors& motor_data)
{
  ScopedLock L(&lock_motors_);
  motors_ = motor_data;
}

}}  // namespace data::hyped
