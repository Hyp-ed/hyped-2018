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
  init();
  readDataFromFile(file_path);
  setData();
}

FakeImu::FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                 NavigationVector gyr_val, NavigationType gyr_noise) :
  acc_val(acc_val), gyr_val(gyr_val),
  acc_noise(acc_noise), gyr_noise(gyr_noise)
{
  init();
  setData();
}

FakeImu::~FakeImu()
{
  if (file.is_open())
    file.close();
}

void FakeImu::init()
{
  /* TODO(Uday): Initialize this
  accPrevReadTime = high_resolution_clock::now();
  gyrPrevReadTime = high_resolution_clock::now();

  prevAccData = ;
  prevGyrData = ;
   */
}

void FakeImu::setData()
{
  imu_.acc.value = acc_val;
  imu_.gyr.value = gyr_val;
}

void FakeImu::getData(Imu* imu)
{
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

bool FakeImu::readDataFromFile(std::string file_path)
{
  if (!file.is_open())
    file.open(file_path);
  return readNextLine();
}

bool FakeImu::readNextLine()
{
  if (file.is_open() && !file.eof()) {
    file >> acc_val[0];
    file >> acc_val[1];
    file >> acc_val[2];

    file >> acc_noise;

    file >> gyr_val[0];
    file >> gyr_val[1];
    file >> gyr_val[2];

    file >> gyr_noise;

    return true;
  } return false;
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
