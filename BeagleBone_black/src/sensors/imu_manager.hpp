/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 19/06/18
 * Description: IMU manager for getting IMU data from around the pod
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

#ifndef BEAGLEBONE_BLACK_SENSORS_IMU_MANAGER_HPP_
#define BEAGLEBONE_BLACK_SENSORS_IMU_MANAGER_HPP_

#include <cstdint>

#include "sensors/manager_interface.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/interface.hpp"
#include "utils/system.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using data::NavigationVector;

namespace sensors {

class ImuManager: public ImuManagerInterface {
 public:
  ImuManager(Logger& log, data::DataPoint<array<Imu, data::Sensors::kNumImus>> *imu);
  void run() override;
  bool updated() override;
  void resetTimestamp() override;
  array<array<NavigationVector, 2>, data::Sensors::kNumImus> getCalibrationData() override;

 private:
  utils::System& sys_;
  data::DataPoint<array<Imu, data::Sensors::kNumImus>> *sensors_imu_;
  data::Data&          data_;

  uint8_t         chip_select_[data::Sensors::kNumImus];
  ImuInterface*   imu_[data::Sensors::kNumImus];
  ImuInterface*   imu_accelerating_[data::Sensors::kNumImus];
  ImuInterface*   imu_decelerating_[data::Sensors::kNumImus];
  array<array<NavigationVector, 2>, data::Sensors::kNumImus> imu_calibrations_;
  bool is_fake_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_IMU_MANAGER_HPP_
