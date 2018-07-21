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
#ifdef PROXI

#include <cstdint>

#include "sensors/manager_interface.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"
#include "utils/system.hpp"
#include "utils/io/i2c.hpp"
#include "utils/math/statistics.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using utils::io::I2C;
using utils::math::OnlineStatistics;

namespace sensors {

class ProxiManager: public ProxiManagerInterface {
  static constexpr uint8_t kMultiplexerAddr = 0x70;
  typedef array<float, data::Sensors::kNumProximities>                      CalibrationArray;
  typedef data::DataPoint<array<Proximity, data::Sensors::kNumProximities>> DataArray;
 public:
  ProxiManager(Logger& log,
               bool isFront,
               DataArray *proxi);
  void run()                            override;
  bool updated()                        override;
  void resetTimestamp()                 override;
  CalibrationArray getCalibrationData() override;

 private:
  DataArray*        sensors_proxi_;
  CalibrationArray  proxi_calibration_;
  ProxiInterface*   proxi_[data::Sensors::kNumProximities];
  I2C&              i2c_;
  bool              is_fake_;
  bool              is_front_;
  OnlineStatistics<float> stats_[data::Sensors::kNumProximities];
  bool              is_calibrated_;
};

}}  // namespace hyped::sensors
#endif
#endif  // BEAGLEBONE_BLACK_SENSORS_PROXI_MANAGER_HPP_
