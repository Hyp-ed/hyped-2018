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

#include <string>
#include "sensors/imu_interface.hpp"
#include "data/data.hpp"

namespace hyped {

using data::Imu;

namespace sensors {

class FakeImu : public ImuInterface {
 public:
  void getData(Imu* imu);
  void addNoiseToData(Imu* imu);
  void readDataFromFile(std::string file_path);
};

void FakeImu::getData(Imu* imu)
{
  // TODO(Uday): Create method
}

void FakeImu::addNoiseToData(Imu* imu)
{
  // TODO(Uday): Create method
}

void FakeImu::readDataFromFile(std::string file_path)
{
  // TODO(Uday): Create method
}

}}  // namespace hyped::sensors
