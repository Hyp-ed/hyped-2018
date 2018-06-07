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
#include <algorithm>
#include <fstream>
#include <string>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/fake_imu.hpp"

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

FakeImu::FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                 NavigationVector gyr_val, NavigationType gyr_noise)
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
      prev_acc = acc_val_read[pt_acc];
      pt_acc++;
      pt_acc = std::min(pt_acc, unsigned(acc_val_read.size()-1));
    }

    if (gyrCheckTime()) {
      prev_gyr = gyr_val_read[pt_gyr];
      pt_gyr++;
      pt_gyr = std::min(pt_gyr, unsigned(gyr_val_read.size()-1));
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

NavigationVector FakeImu::addNoiseToData(NavigationVector value, NavigationType noise)
{
  NavigationVector temp;
  std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<NavigationType> distribution(value[i], noise);
    temp[i] = distribution(generator);
  }

  return temp;
}

void FakeImu::readDataFromFile(std::string acc_file_path, std::string gyr_file_path)
{
  std::ifstream file;
  uint32_t timestamp;
  NavigationVector temp;

  file.open(acc_file_path);

  file >> acc_noise;
  while (file.is_open() && !file.eof()) {
    file >> timestamp >> temp[0] >> temp[1] >> temp[2];
    acc_val_read.push_back(DataPoint<NavigationVector>(timestamp, addNoiseToData(temp, acc_noise)));
  }

  if (file.is_open())
    file.close();

  file.open(gyr_file_path);
  file >> gyr_noise;
  while (file.is_open() && !file.eof()) {
    file >> timestamp >> temp[0] >> temp[1] >> temp[2];
    gyr_val_read.push_back(DataPoint<NavigationVector>(timestamp, addNoiseToData(temp, gyr_noise)));
  }
}

bool FakeImu::accCheckTime()
{
    high_resolution_clock::time_point now = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(now - imu_ref_time);

    if (time_span.count() < 0.000250*acc_count) {
        return false;
    }

    acc_count++;
    return true;
}

bool FakeImu::gyrCheckTime()
{
    high_resolution_clock::time_point now = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(now - imu_ref_time);

    if (time_span.count() < 0.000125*gyr_count) {
        return false;
    }

    gyr_count++;
    return true;
}

}}  // namespace hyped::sensors
