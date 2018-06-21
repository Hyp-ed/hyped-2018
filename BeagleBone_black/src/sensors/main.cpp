/*
 * Author: Martin Kristien and Jack Horsburgh
 * Organisation: HYPED
 * Date: 13/03/18
 * Description:
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

#include "sensors/main.hpp"

#include "data/data.hpp"
#include "sensors/imu_manager.hpp"
#include "sensors/bms_manager.hpp"
#include "sensors/proxi_manager.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using data::Batteries;
using data::StripeCounter;

namespace sensors {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      imu_manager_(new ImuManager(log)),
      proxi_manager_front_(new ProxiManager(log, true)),
      proxi_manager_back_(new ProxiManager(log, false)),
      battery_manager_lp_(new BmsManager(log))
{
  // Config new IMU manager
  imu_manager_->config(&sensors_.imu);

  // Config Proxi manager
  proxi_manager_front_->config(&sensors_.proxi_front);
  proxi_manager_back_->config(&sensors_.proxi_back);

  // Config BMS Manager
  battery_manager_lp_->config(&batteries_.low_power_batteries);

  // Used for initialisation of old sensor and old battery data
  old_imu_timestamp_ = sensors_.imu.timestamp;

  // @TODO (Ragnor) Add second Keyence?
  // create Keyence
  keyence = new Keyence(log_, 73);
  keyence->start();

  old_proxi_back_timestamp = sensors_.proxi_back.timestamp;
  old_proxi_front_timestamp = sensors_.proxi_front.timestamp;
  old_batteries_ = batteries_;
}

void Main::run()
{
  while (1) {
    // Write sensor data to data structure only when all the imu and proxi values are different
    if (imu_manager_->updated()) {
      data_.setSensorsData(sensors_);
    }

    if (proxi_manager_front_->updated() && proxi_manager_back_->updated()) {
      data_.setSensorsData(sensors_);
    }

    // Update battery data only when there is some change
    if (battery_manager_lp_->updated()) {
      data_.setBatteryData(batteries_);
      old_batteries_ = batteries_;
    }
    data_.setStripeCounterData(keyence->getStripeCounter());
    yield();
  }
}

}}  // namespace hyped::sensors
