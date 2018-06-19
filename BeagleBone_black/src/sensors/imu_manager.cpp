/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 19/06/18
 * Description: IMU manager for getting IMU data from around the pod
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


#include "sensors/imu_manager.hpp"

#include "sensors/mpu9250.hpp"
#include "data/data.hpp"

namespace hyped {

using data::Data;
using data::Sensors;

namespace sensors {

ImuManager::ImuManager(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      chip_select_ {31, 50, 48, 51}
{
  // create IMUs, might consider using fake_imus based on input arguments
  for (int i = 0; i < data::Sensors::kNumImus; i++) {
    imu_[i] = new MPU9250(log, 60, 0x08, 0x00);;
  }
}

void ImuManager::run()
{
  while (1) {
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i]->getData(&((*sensors_imu_)[i]));
    }
  }
}

void ImuManager::config(array<Imu, data::Sensors::kNumImus> *imu)
{
  sensors_imu_ = imu;
}
}}  // namespace hyped::sensors