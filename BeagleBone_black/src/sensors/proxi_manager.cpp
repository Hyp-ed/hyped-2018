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

#ifdef PROXI
#include "sensors/proxi_manager.hpp"

#include "sensors/can_proxi.hpp"
#include "sensors/vl6180.hpp"
#include "data/data.hpp"
#include "utils/timer.hpp"
#include "utils/io/i2c.hpp"
#include "sensors/fake_proxi.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using utils::System;
using sensors::FakeProxi;
using utils::io::I2C;
using utils::math::OnlineStatistics;


namespace sensors {

ProxiManager::ProxiManager(Logger& log,
                           bool is_front,
                           ProxiManager::DataArray *proxi)
    : ProxiManagerInterface(log),
      sensors_proxi_(proxi),
      i2c_(I2C::getInstance()),
      is_front_(is_front),
      is_calibrated_(false)
{
  System& sys = System::getSystem();
  old_timestamp_ = utils::Timer::getTimeMicros();
  if (sys.fake_proxi || sys.fake_sensors) is_fake_ = true;

  if (is_fake_) {
    // TODO(anyone) add read to file after
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      FakeProxi* proxi = new FakeProxi(log_, 23, 1.5, true);
      proxi_[i] = proxi;
    }
  } else if (is_front_) {
    // create real proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      i2c_.write(kMultiplexerAddr, 0x01 << i);
      VL6180* proxi = new VL6180(0x29, log_);
      proxi_[i] = proxi;
    }
  } else {
    // create CAN-based proximities
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      CanProxi* proxi = new CanProxi(i, log_);
      proxi_[i] = proxi;
    }
  }

  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    stats_[i] = OnlineStatistics<float>();
  }
}

void ProxiManager::run()
{
  // collect calibration data
  uint32_t calib_counter = 0;
  Proximity proxi;
  while (!is_calibrated_) {
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      if (is_front_) {
        i2c_.write(kMultiplexerAddr, 0x01 << i);
      }
      proxi_[i]->startRanging();
    }

    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      if (is_front_) {
        if (!i2c_.write(kMultiplexerAddr, 0x01 << i)) {
          if (!is_fake_)log_.ERR("Proxi-Manager", "No Multiplexer connection");
        }
      }
      proxi_[i]->getData(&proxi);
      if (proxi.operational) stats_[i].update(proxi.val);
    }
    calib_counter++;
    if (calib_counter >= 100) is_calibrated_ = true;
  }
  log_.INFO("PROXI-MANAGER", "Calibration complete!");

  // collect real data
  while (1) {
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      if (is_front_) {
        i2c_.write(kMultiplexerAddr, 0x01 << i);
      }
      proxi_[i]->startRanging();
    }
    for (int i = 0; i < data::Sensors::kNumProximities; i++) {
      if (is_front_) {
        i2c_.write(kMultiplexerAddr, 0x01 << i);
      }
      proxi_[i]->getData(&(sensors_proxi_->value[i]));
    }
    sensors_proxi_->timestamp = utils::Timer::getTimeMicros();
  }
}

ProxiManager::CalibrationArray ProxiManager::getCalibrationData()
{
  while (!is_calibrated_) {
    Thread::yield();
  }
  for (int i = 0; i < data::Sensors::kNumProximities; i++) {
    proxi_calibration_[i] = stats_[i].getVariance();
  }
  return proxi_calibration_;
}

bool ProxiManager::updated()
{
  if (old_timestamp_ != sensors_proxi_->timestamp) {
    return true;
  }
  return false;
}

void ProxiManager::resetTimestamp()
{
  old_timestamp_ = sensors_proxi_->timestamp;
}
}}  // namespace hyped::sensors
#endif
