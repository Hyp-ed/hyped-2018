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

using std::chrono::duration;
using std::chrono::duration_cast;

namespace hyped {
namespace sensors {

FakeImu::FakeImu(std::string file_path)
{
  readFromFile = true;
  readDataFromFile(file_path);
  setData();
  init();
}

FakeImu::FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                 NavigationVector gyr_val, NavigationType gyr_noise) :
  acc_val(acc_val), gyr_val(gyr_val),
  acc_noise(acc_noise), gyr_noise(gyr_noise)
{
  readFromFile = false;
  setData();
  init();
}

void FakeImu::init()
{
  accPrevReadTime = high_resolution_clock::now();
  gyrPrevReadTime = high_resolution_clock::now();

  prevAccData = DataPoint<NavigationVector>(imu_.acc.timestamp,
                                            imu_.acc.value + addNoiseToData(acc_val, acc_noise));
  prevGyrData = DataPoint<NavigationVector>(imu_.gyr.timestamp,
                                            imu_.gyr.value + addNoiseToData(gyr_val, gyr_noise));
}

void FakeImu::setData()
{
  imu_.acc.value = acc_val;
  imu_.gyr.value = gyr_val;
}

void FakeImu::getData(Imu* imu)
{
  if (readFromFile == true) {
    if (filePointerAcc == acc_val_read.size()) {
      // TODO(Uday): Set the acc sensor state to offline
    } else {
      prevAccData = acc_val_read[filePointerAcc];
      filePointerAcc++;
    }

    if (filePointerGyr == gyr_val_read.size()) {
      // TODO(Uday): Set the gyr sensor state to offline
    } else {
      prevGyrData = gyr_val_read[filePointerGyr];
      filePointerGyr++;
    }

    return;
  }

  if (accCheckTime())
    prevAccData = DataPoint<NavigationVector>(imu_.acc.timestamp,
                                           imu_.acc.value + addNoiseToData(acc_val, acc_noise));

  if (gyrCheckTime())
    prevGyrData = DataPoint<NavigationVector>(imu_.gyr.timestamp,
                                           imu_.gyr.value + addNoiseToData(gyr_val, gyr_noise));

  imu->acc = prevAccData;
  imu->gyr = prevGyrData;
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
  if (!file.is_open())
    file.open(file_path);

  uint32_t acc_timestamp, gyr_timestamp;
  NavigationVector acc_val_temp, gyr_val_temp;
  while (file.is_open() && !file.eof()) {
    file >> acc_timestamp;
    file >> acc_val_temp[0];
    file >> acc_val_temp[1];
    file >> acc_val_temp[2];

    file >> gyr_timestamp;
    file >> gyr_val_temp[0];
    file >> gyr_val_temp[1];
    file >> gyr_val_temp[2];

    acc_val_read.push_back(DataPoint<NavigationVector>(acc_timestamp, acc_val_temp));
    gyr_val_read.push_back(DataPoint<NavigationVector>(gyr_timestamp, gyr_val_temp));
  }

  if (file.is_open())
    file.close();
}

bool FakeImu::accCheckTime()
{
    high_resolution_clock::time_point now = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(now - accPrevReadTime);

    if (time_span.count() < 0.000250) {
        return false;
    }

    accPrevReadTime = now;
    return true;
}

bool FakeImu::gyrCheckTime()
{
    high_resolution_clock::time_point now = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(now - gyrPrevReadTime);

    if (time_span.count() < 0.000125) {
        return false;
    }

    gyrPrevReadTime = now;
    return true;
}

}}  // namespace hyped::sensors
