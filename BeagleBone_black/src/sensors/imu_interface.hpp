/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main interface for IMU class.
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

#ifndef BEAGLEBONE_BLACK_SENSORS_IMU_INTERFACE_HPP_
#define BEAGLEBONE_BLACK_SENSORS_IMU_INTERFACE_HPP_

#include <string>
#include "data/data.hpp"

namespace hyped {

using data::Imu;

namespace sensors {

class ImuInterface {
 public:
  virtual void getData(Imu* imu) = 0;
  virtual void addNoiseToData(Imu* imu) = 0;
  virtual void readDataFromFile(std::string file_path) = 0;
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_IMU_INTERFACE_HPP_
