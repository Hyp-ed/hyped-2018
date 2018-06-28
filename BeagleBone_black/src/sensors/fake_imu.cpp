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
#include "utils/timer.hpp"


namespace hyped {
namespace sensors {

FakeImuAccelerating::FakeImuAccelerating(utils::Logger& log,
                                         std::string acc_file_path,
                                         std::string gyr_file_path)
    : log_(log),
      acc_val_(0),
      gyr_val_(0),
      acc_noise_(1),
      gyr_noise_(1),
      acc_file_path_(acc_file_path),
      gyr_file_path_(gyr_file_path),
      is_started_(false)

{
  read_file_ = true;
  readDataFromFile(acc_file_path_, gyr_file_path_);
  log_.INFO("Fake-IMU-accl", "Fake IMU initialised");
}

void FakeImuAccelerating::start()
{
  imu_ref_time_ = utils::Timer::getTimeMicros();
  acc_count_ = gyr_count_ = 0;
}

void FakeImuAccelerating::getData(Imu* imu)
{
  if (!is_started_) {
    is_started_ = true;
    start();
  }
  if (accCheckTime()) {
    acc_count_ = std::min(acc_count_/kAccTimeInterval, (int64_t) acc_val_read_.size());
    // Check so you don't go out of bounds
    if (acc_count_ == (int64_t) acc_val_read_.size()) {
      prev_acc_ = acc_val_read_[acc_count_-1];
    } else {
      prev_acc_ = acc_val_read_[acc_count_];
    }
  }

  if (gyrCheckTime()) {
    gyr_count_ = std::min(gyr_count_/kGyrTimeInterval, (int64_t) gyr_val_read_.size());
    // Check so you don't go out of bounds
    if (gyr_count_ ==  (int64_t) gyr_val_read_.size()) {
      prev_gyr_ = gyr_val_read_[gyr_count_-1];
    } else {
      prev_gyr_ = gyr_val_read_[gyr_count_];
    }
  }
  imu->acc = prev_acc_;
  imu->gyr = prev_gyr_;
}

NavigationVector FakeImuAccelerating::addNoiseToData(NavigationVector value, NavigationVector noise)
{
  NavigationVector temp;
  static std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<NavigationType> distribution(value[i], noise[i]);
    temp[i] = distribution(generator);
  }
  return temp;
}

void FakeImuAccelerating::readDataFromFile(std::string acc_file_path, std::string gyr_file_path)
{
  for (int i = 0; i < 2; i++) {
    std::string file_path;
    uint32_t timestamp;
    std::vector<NavigationVector>* val_read;

    if (i == 0) {
      file_path = acc_file_path;
      timestamp = kAccTimeInterval;
      val_read  = &acc_val_read_;
    } else {
      file_path = gyr_file_path;
      timestamp = kGyrTimeInterval;
      val_read  = &gyr_val_read_;
    }

    std::ifstream file;
    file.open(file_path);
    if (!file.is_open()) {
      log_.ERR("Fake-IMU-accl", "Wrong file path for argument: %d", i);
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
        log_.ERR("Fake-IMU-accl", "Timestamp format invalid %d", temp_time);
      }

      int value_counter = 0;
      while (input >> temp_value[value_counter] && value_counter < 10) {
        value_counter++;
      }

      if (value_counter != 6) {
        log_.ERR("Fake-IMU-accl", "Incomplete values for the argument timestamp: %d", temp_time);
      }

      for (int i = 0; i < 3; i++)
        value[i] = temp_value[i];
      for (int i = 0; i < 3; i++)
        noise[i] = temp_value[i+3];

      val_read->push_back(addNoiseToData(value, noise));

      counter++;
    }

    file.close();
  }
}

bool FakeImuAccelerating::accCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = now - imu_ref_time_;

  if (time_span < kAccTimeInterval*acc_count_) {
    return false;
  }
  acc_count_ = time_span/kAccTimeInterval + 1;
  return true;
}

bool FakeImuAccelerating::gyrCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = now - imu_ref_time_;

  if (time_span < kGyrTimeInterval*gyr_count_) {
    return false;
  }

  gyr_count_ = time_span/kGyrTimeInterval + 1;
  return true;
}

FakeImuStationary::FakeImuStationary(utils::Logger& log,
                                     NavigationVector acc_val,
                                     NavigationVector acc_noise,
                                     NavigationVector gyr_val,
                                     NavigationVector gyr_noise)
    : log_(log),
      acc_val_(acc_val),
      gyr_val_(gyr_val),
      acc_noise_(acc_noise),
      gyr_noise_(gyr_noise)
{
  imu_ref_time_ = utils::Timer::getTimeMicros();
  acc_count_ = gyr_count_ = 0;
  log_.INFO("Fake-IMU-stationary", "Stationary IMU initialised");
}

void FakeImuStationary::getData(Imu* imu)
{
  if (accCheckTime()) {
    prev_acc_ = addNoiseToData(acc_val_, acc_noise_);
  }
  if (gyrCheckTime()) {
    prev_gyr_ = addNoiseToData(gyr_val_, gyr_noise_);
  }
  imu->acc = prev_acc_;
  imu->gyr = prev_gyr_;
}

NavigationVector FakeImuStationary::addNoiseToData(NavigationVector value, NavigationVector noise)
{
  NavigationVector temp;
  static std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<NavigationType> distribution(value[i], noise[i]);
    temp[i] = distribution(generator);
  }
  return temp;
}

bool FakeImuStationary::accCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = now - imu_ref_time_;

  if (time_span < kAccTimeInterval*acc_count_) {
    return false;
  }
  acc_count_ = time_span/kAccTimeInterval + 1;
  return true;
}

bool FakeImuStationary::gyrCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = now - imu_ref_time_;

  if (time_span < kGyrTimeInterval*gyr_count_) {
    return false;
  }

  gyr_count_ = time_span/kGyrTimeInterval + 1;
  return true;
}

}}  // namespace hyped::sensors
