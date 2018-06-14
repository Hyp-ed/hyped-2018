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
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/fake_imu.hpp"

using std::chrono::microseconds;
using std::chrono::duration;
using std::chrono::duration_cast;

namespace hyped {
namespace sensors {

FakeImu::FakeImu(std::string acc_file_path, std::string gyr_file_path)
    : pt_acc(0), pt_gyr(0)
{
  read_file = true;
  readDataFromFile(acc_file_path, gyr_file_path);
  setData();
}

FakeImu::FakeImu(NavigationVector acc_val, NavigationVector acc_noise,
                 NavigationVector gyr_val, NavigationVector gyr_noise)
    : acc_val(acc_val), gyr_val(gyr_val),
      acc_noise(acc_noise), gyr_noise(gyr_noise)
{
  read_file = false;
  setData();
}

void FakeImu::setData()
{
  imu_ref_time = high_resolution_clock::now();
  acc_count = gyr_count = 0;
}

void FakeImu::getData(Imu* imu)
{
  if (read_file == true) {
    if (accCheckTime()) {
      acc_count = std::min(acc_count, unsigned(acc_val_read.size()));
      prev_acc = acc_val_read[acc_count-1];
    }

    if (gyrCheckTime()) {
      gyr_count = std::min(gyr_count, unsigned(gyr_val_read.size()));
      prev_gyr = gyr_val_read[gyr_count-1];
    }
  } else {
    if (accCheckTime())
      prev_acc = DataPoint<NavigationVector>(kAccTimeInterval*(acc_count-1),
                                                addNoiseToData(acc_val, acc_noise));

    if (gyrCheckTime())
      prev_gyr = DataPoint<NavigationVector>(kGyrTimeInterval*(gyr_count-1),
                                                addNoiseToData(gyr_val, gyr_noise));
  }

  imu->acc = prev_acc;
  imu->gyr = prev_gyr;
}

NavigationVector FakeImu::addNoiseToData(NavigationVector value, NavigationVector noise)
{
  NavigationVector temp;
  std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<NavigationType> distribution(value[i], noise[i]);
    temp[i] = distribution(generator);
  }

  return temp;
}

void FakeImu::readDataFromFile(std::string acc_file_path, std::string gyr_file_path)
{
  for (int i = 0; i < 2; i++) {
    std::string file_path;
    uint32_t timestamp;
    std::vector<DataPoint<NavigationVector>>* val_read;

    if (i == 0) {
      file_path = acc_file_path;
      timestamp = kAccTimeInterval;
      val_read  = &acc_val_read;
    } else {
      file_path = gyr_file_path;
      timestamp = kGyrTimeInterval;
      val_read  = &gyr_val_read;
    }

    std::ifstream file;
    file.open(file_path);
    if (!file.is_open()) {
      throw std::invalid_argument("Wrong file path for argument " + i);
    }

    NavigationVector value, noise;
    double temp_value[10];
    int counter = 0;
    uint32_t temp_time;
    std::string line;

    while (getline(file, line)) {
      std::stringstream input(line);
      input >> temp_time;

      if (temp_time != timestamp*counter) {
        throw std::invalid_argument("Timestamp format invalid");
      }

      int value_counter = 0;
      while (input >> temp_value[value_counter] && value_counter < 10) {
        value_counter++;
      }

      if (value_counter != 6) {
        throw std::invalid_argument("Incomplete values for the argument timestamp " + temp_time);
      }

      for (int i = 0; i < 3; i++)
        value[i] = temp_value[i];
      for (int i = 0; i < 3; i++)
        noise[i] = temp_value[i+3];

      val_read->push_back(DataPoint<NavigationVector>(temp_time, addNoiseToData(value, noise)));

      counter++;
    }

    file.close();
  }
}

bool FakeImu::accCheckTime()
{
  high_resolution_clock::time_point now = high_resolution_clock::now();
  microseconds time_span = duration_cast<microseconds>(now - imu_ref_time);

  if (unsigned(time_span.count()) < kAccTimeInterval*acc_count) {
    return false;
  }

  acc_count = time_span.count()/kAccTimeInterval + 1;
  return true;
}

bool FakeImu::gyrCheckTime()
{
  high_resolution_clock::time_point now = high_resolution_clock::now();
  microseconds time_span = duration_cast<microseconds>(now - imu_ref_time);

  if (unsigned(time_span.count()) < kGyrTimeInterval*gyr_count) {
    return false;
  }

  gyr_count = time_span.count()/kGyrTimeInterval + 1;
  return true;
}

}}  // namespace hyped::sensors
