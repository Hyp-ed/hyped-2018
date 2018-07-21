/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main class for fake Proxis
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

// TODO(Uday): Change the units

#ifndef BEAGLEBONE_BLACK_SENSORS_FAKE_PROXI_HPP_
#define BEAGLEBONE_BLACK_SENSORS_FAKE_PROXI_HPP_
#ifdef PROXI
#include <string>
#include <vector>

#include "sensors/interface.hpp"
#include "utils/logger.hpp"

using std::chrono::high_resolution_clock;

namespace hyped {

using data::DataPoint;

namespace sensors {

class FakeProxi : public ProxiInterface {
 public:
  /*
   * @brief    A constructor for the fake proximity class by reading from file
   */
  FakeProxi(utils::Logger& log, std::string file_path);

  /*
   * @brief    A constructor for the fake proximity class by generating random data
   */
  FakeProxi(utils::Logger& log, uint8_t value, float noise, bool operational);

  /*
   * @brief    A function to check if the proximity sensor is online
   */
  bool isOnline() override { return true; }

  void startRanging() override;

  /*
   * @brief    A function to get the proximity data
   */
  void getData(Proximity* proxi) override;


 private:
  const uint8_t kProxiTimeInterval = 10;

  /*
   * @brief    A function to load data from file to vector. The file format of the input
   *           file has to be the following for each line
   *
   *              timestamp  value  noise
   */
  void readDataFromFile(std::string file_path);

  /*
   * @brief    A function to add noise to the proximity data
   */
  uint8_t addNoiseToData(uint8_t value, float noise);

  /*
   * @brief    Checks to see if sufficient time has pass for the sensor to be updated
   */
  bool checkTime();

  /*
   * @brief    Initializes the shared variables
   */
  void setData();

  bool read_file_;

  uint8_t value_;
  float noise_;
  bool operational_;
  DataPoint<uint8_t> prev_reading_;

  int64_t reading_counter_;
  std::vector<DataPoint<uint8_t>> val_read_;
  std::vector<bool> val_operational_;
  high_resolution_clock::time_point ref_time_;
  utils::Logger& log_;
};

}}  // namespace hyped::sensors

#endif
#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_PROXI_HPP_
