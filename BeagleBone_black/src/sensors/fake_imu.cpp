/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main class for fake IMUs.
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

#include <random>

#include <fstream>
#include <string>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/fake_imu.hpp"

namespace hyped {

using data::Imu;
using data::NavigationType;
using data::NavigationVector;
using data::DataPoint;

namespace sensors {

FakeImu::FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                 NavigationVector gyr_val, NavigationType gyr_noise) :
  acc_val(acc_val), gyr_val(gyr_val),
  acc_noise(acc_noise), gyr_noise(gyr_noise)
{
  setData();
}

void FakeImu::setData()
{
  imu_.acc.value = acc_val;
  imu_.gyr.value = gyr_val;
}

Imu FakeImu::getData()
{
  Imu temp;
  temp.acc = DataPoint<NavigationVector>(imu_.acc.timestamp,
                                         imu_.acc.value + addNoiseToData(acc_val, acc_noise));
  temp.gyr = DataPoint<NavigationVector>(imu_.gyr.timestamp,
                                         imu_.gyr.value + addNoiseToData(gyr_val, gyr_noise));
  return temp;
}

NavigationVector FakeImu::addNoiseToData(NavigationVector value, NavigationType noise)
{
  NavigationVector temp;
  std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<double> distribution(value[i], noise);
    temp[i] = distribution(generator);
  }

  return temp;
}

void FakeImu::readDataFromFile(std::string file_path)
{
  std::ifstream file;
  file.open(file_path);
  while (!file.eof()) {
    // TODO(Uday): Implement this
  }
  file.close();
}

}}  // namespace hyped::sensors
