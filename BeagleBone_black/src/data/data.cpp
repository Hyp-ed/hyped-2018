/*
 * Organisation: HYPED
 * Date: 18/02/2018
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

#include "data.hpp"

namespace hyped {
namespace data {

Data& Data::getInstance()
{
  static Data d;
  return d;
}

StateMachine Data::getStateMachineData() const
{
  return state_machine_;
}

void Data::setStateMachineData(const StateMachine& sm_data)
{
  state_machine_ = sm_data;
}

Navigation Data::getNavigationData() const
{
  return navigation_;
}

void Data::setNavigationData(const Navigation& nav_data)
{
  navigation_ = nav_data;
}

Sensors Data::getSensorsData() const
{
  return sensors_;
}

void Data::setSensorsData(const Sensors& sensors_data)
{
  sensors_ = sensors_data;
}

Motors Data::getMotorData() const
{
  return motors_;
}

void Data::setMotorData(const Motors& motor_data)
{
  motors_ = motor_data;
}

}}  // namespace hyped::data
