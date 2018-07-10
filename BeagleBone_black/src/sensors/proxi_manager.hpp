/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 19/06/18
 * Description: Proxi manager for getting proxi data from around the pod
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

#ifndef BEAGLEBONE_BLACK_SENSORS_PROXI_MANAGER_HPP_
#define BEAGLEBONE_BLACK_SENSORS_PROXI_MANAGER_HPP_

#include <cstdint>

#include "sensors/manager_interface.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"
#include "utils/io/i2c.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using utils::io::I2C;

namespace sensors {

class ProxiManager: public ProxiManagerInterface {
  static constexpr uint8_t kMultiplexerAddr = 0x70;
 public:
  ProxiManager(Logger& log,
               bool isFront,
               data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> *proxi);
  void run() override;
  bool updated() override;
  void resetTimestamp() override;
  array<float, data::Sensors::kNumProximities> getCalibrationData() override;

 private:
  data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> *sensors_proxi_;
  ProxiInterface* proxi_[data::Sensors::kNumProximities];
  array<float, data::Sensors::kNumProximities> proxi_calibration_;
  I2C& i2c_;
  bool is_front_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_PROXI_MANAGER_HPP_
