/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main class for fake proximity.
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
#ifdef PROXI
#include "sensors/fake_proxi.hpp"

#include <random>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <ctime>
#include <cstdlib>

#include "data/data_point.hpp"
#include "utils/math/statistics.hpp"
#include "utils/concurrent/thread.hpp"

using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::duration_cast;

namespace hyped {

using utils::math::OnlineStatistics;
using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

FakeProxi::FakeProxi(Logger& log, std::string file_path)
    : read_file_(true),
      log_(log)
{
  readDataFromFile(file_path);
  setData();
}

FakeProxi::FakeProxi(Logger& log, uint8_t value, float noise, bool operational)
    : read_file_(false),
      value_(value),
      noise_(noise),
      operational_(operational),
      log_(log)
{
  setData();
  log_.INFO("Fake-Proxi", "Initialised");
}

void FakeProxi::setData()
{
  ref_time_ = high_resolution_clock::now();
  reading_counter_ = 0;
  srand(time(NULL));
}

void FakeProxi::startRanging() {}

void FakeProxi::getData(Proximity* proxi)
{
  bool operational = false;
  Thread::sleep(10);
  bool update_time = checkTime();
  reading_counter_ = std::min(reading_counter_, (int64_t) val_read_.size());
  if (read_file_ && update_time) {
    prev_reading_ = val_read_[reading_counter_-1];
    operational = val_operational_[reading_counter_-1];
  } else if (update_time) {
    prev_reading_ = DataPoint<uint8_t>(kProxiTimeInterval*(reading_counter_-1),
                                      addNoiseToData(value_, noise_));
    operational = operational_;
  }

  // TODO(Anyone): Add timestamp
  proxi->val = prev_reading_.value;
  proxi->operational = operational;
}

void FakeProxi::readDataFromFile(std::string file_path)
{
  std::ifstream file;
  file.open(file_path);
  if (!file.is_open()) {
    throw std::invalid_argument("Wrong file path");
  }

  int8_t timestamp = kProxiTimeInterval;
  int8_t counter;
  int32_t time_counter = 0;
  int temp[6];
  std::string line;
  while (getline(file, line)) {
    std::stringstream input(line);

    counter = 0;
    while (input >> temp[counter] && counter < 4) {
      counter++;
    }

    if (counter != 4) {
      throw std::invalid_argument("Incomplete values for the argument line");
    }

    if (temp[0] != timestamp*time_counter) {
      throw std::invalid_argument("Timestamp value incorrect");
    }
    val_read_.push_back(DataPoint<uint8_t>(temp[0], addNoiseToData(temp[1], temp[2])));
    val_operational_.push_back(temp[3]);
    time_counter++;
  }

  file.close();
}

uint8_t FakeProxi::addNoiseToData(uint8_t value, float noise)
{
  float temp;
  static std::default_random_engine generator;

  std::normal_distribution<float> distribution(static_cast<float>(value), noise);
  temp = distribution(generator);
  if (temp > 255.0) {
    temp = 255.0;
  } else if (temp < 0) {
    temp = 0;
  }
  return static_cast<uint8_t>(temp);
}

bool FakeProxi::checkTime()
{
  high_resolution_clock::time_point now = high_resolution_clock::now();
  milliseconds time_span = duration_cast<milliseconds>(now - ref_time_);

  if (time_span.count() < kProxiTimeInterval*reading_counter_) {
    return false;
  }

  reading_counter_ = time_span.count()/kProxiTimeInterval + 1;
  return true;
}

}}   // namespace hyped::sensors
#endif
