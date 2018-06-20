/*
 * Author: Martin Kristien and Jack Horsburgh
 * Organisation: HYPED
 * Date: 13/03/18
 * Description:
 * Main manages sensor drivers, collects data from sensors and updates
 * shared Data::Sensors structure. Main is not responsible for initialisation
 * of supporting io drivers (i2c, spi, can). This should be done by the sensor
 * drivers themselves.
 * Currently supported sensors:
 * - BMS (low powered), ids: 0
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

#ifndef BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_
#define BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_

#include <cstdint>

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/imu_manager.hpp"
#include "sensors/interface.hpp"
#include "sensors/proxi_manager.hpp"
#include "sensors/bms_manager.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

class CANProxi;

class Main: public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  void run() override;

 private:
  bool updateImu();
  bool updateProxi();
  bool updateBattery();
  data::Data&     data_;

  // master data structures
  data::Sensors   sensors_;
  data::Batteries batteries_;

  // Previous data
  uint64_t old_imu_timestamp_[data::Sensors::kNumImus];
  uint64_t old_proxi_back_timestamp;
  uint64_t old_proxi_front_timestamp;
  data::Batteries old_batteries_;

  ImuManager imu_manager_;
  ProxiManager proxi_manager_front_;
  ProxiManager proxi_manager_back_;
  BmsManager  battery_manager_lp;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_
