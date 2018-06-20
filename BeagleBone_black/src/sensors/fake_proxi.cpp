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

#include "sensors/fake_proxi.hpp"

#include <random>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

#include "data/data_point.hpp"

using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::duration_cast;

namespace hyped {
namespace sensors {

FakeProxi::FakeProxi(std::string file_path)
    : read_file_(true), reading_counter_(0)
{
  readDataFromFile(file_path);
  setData();
}

FakeProxi::FakeProxi(uint8_t value, uint8_t noise)
    : read_file_(false), value_(value), noise_(noise)
{
  setData();
}

void FakeProxi::setData()
{
  ref_time_ = high_resolution_clock::now();
}

void FakeProxi::getData(Proximity* proxi)
{
  bool update_time = checkTime();
  if (read_file_ && update_time) {
    reading_counter_ = std::min(reading_counter_, (int64_t) val_read_.size());
    prev_reading_ = val_read_[reading_counter_-1];
  } else if (update_time) {
    prev_reading_ = DataPoint<uint8_t>(kProxiTimeInterval*(reading_counter_-1),
                                      addNoiseToData(value_, noise_));
  }

  // TODO(Anyone): Add timestamp
  proxi->val = prev_reading_.value;
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
  int temp[4];
  std::string line;
  while (getline(file, line)) {
    std::stringstream input(line);

    counter = 0;
    while (input >> temp[counter] && counter < 4) {
      counter++;
    }

    if (counter != 3) {
      throw std::invalid_argument("Incomplete values for the argument line");
    }

    if (temp[0] != timestamp*time_counter) {
      throw std::invalid_argument("Timestamp value incorrect");
    }

    val_read_.push_back(DataPoint<uint8_t>(temp[0], addNoiseToData(temp[1], temp[2])));
    time_counter++;
  }

  file.close();
}

uint8_t FakeProxi::addNoiseToData(uint8_t value, double noise)
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(value, noise);

  double ans = distribution(generator);
  if (ans > 255)
    return 255;

  if (ans < 0)
    return 0;

  return ans;
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

