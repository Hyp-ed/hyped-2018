/*
 * Author: Martin Kristien, Jack Horsburgh and Ragnor Comerford
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

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "data/data.hpp"
#include "sensors/imu_manager.hpp"
#include "sensors/bms_manager.hpp"
#include "sensors/proxi_manager.hpp"
#include "sensors/fake_gpio_counter.hpp"
#include "sensors/gpio_counter.hpp"

constexpr float kWheelDiameter = 0.08;   // TODO(anyone) Get wheel radius for optical encoder
namespace hyped {

using data::Data;
using data::Sensors;
using data::Batteries;
using data::StripeCounter;
using data::SensorCalibration;
using utils::System;

namespace sensors {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      sys_(System::getSystem()),
      imu_manager_(new ImuManager(log, &sensors_.imu)),
      proxi_manager_front_(new ProxiManager(log, true, &sensors_.proxi_front)),
      proxi_manager_back_(new ProxiManager(log, false, &sensors_.proxi_back)),
      battery_manager_(new BmsManager(log,
                                         &batteries_.low_power_batteries,
                                         &batteries_.high_power_batteries)),
      sensor_init_(false),
      battery_init_(false)
{
  // @TODO (Ragnor) Add second Keyence?
  if (sys_.fake_sensors || sys_.fake_keyence) {
    keyence_ = new FakeGpioCounter(log, "../BeagleBone_black/data/in/fake_keyence_input.txt");
    optical_encoder_ = new FakeGpioCounter(log, "../BeagleBone_black/data/in/fake_keyence_input.txt"); //NOLINT
  } else {
    // Pins for keyence GPIO_73 and GPIO_75
    keyence_ = new GpioCounter(log, 73);
    optical_encoder_ = new GpioCounter(log, 76);
  }
}

void Main::run()
{
  // start all managers
  keyence_->start();
  optical_encoder_->start();
  imu_manager_->start();
  proxi_manager_front_->start();
  proxi_manager_back_->start();
  battery_manager_->start();

  // init loop
  while (!sensor_init_) {
    if (imu_manager_->updated() && proxi_manager_front_->updated() && proxi_manager_back_->updated()) { //NOLINT
      sensors_.module_status = data::ModuleStatus::kInit;
      data_.setSensorsData(sensors_);

      // Get calibration data
      SensorCalibration sensor_calibration_data;
      sensor_calibration_data.proxi_front_variance = proxi_manager_front_->getCalibrationData();
      sensor_calibration_data.proxi_back_variance  = proxi_manager_back_->getCalibrationData();
      sensor_calibration_data.imu_variance         = imu_manager_->getCalibrationData();
      data_.setCalibrationData(sensor_calibration_data);
      sensor_init_ = true;

      break;
    }
    yield();
  }
  log_.INFO("SENSORS", "sensors data has been initialised");
  while (!battery_init_) {
    if (battery_manager_->updated()) {
      batteries_.module_status = data::ModuleStatus::kInit;
      data_.setBatteryData(batteries_);
      battery_init_ = true;
      break;
    }
    yield();
  }
  log_.INFO("SENSORS", "batteries data has been initialised");

  // if (battery_init_ && sensor_init_) sensors_.module_status = data::ModuleStatus::kReady;

  // work loop
  while (1) {
    // Write sensor data to data structure only when all the imu or proxi values are different
    if (imu_manager_->updated()) {
      sensors_.keyence_stripe_counter = keyence_->getStripeCounter();
      sensors_.optical_enc_distance = optical_encoder_->getStripeCounter().count.value *
                                      M_PI *
                                      kWheelDiameter;
      data_.setSensorsData(sensors_);
      // Update manager timestamp with a function
      imu_manager_->resetTimestamp();
    }

    if (proxi_manager_front_->updated() && proxi_manager_back_->updated()) {
      data_.setSensorsData(sensors_);
      proxi_manager_front_->resetTimestamp();
      proxi_manager_back_->resetTimestamp();
    }

    // Update battery data only when there is some change
    if (battery_manager_->updated()) {
      data_.setBatteryData(batteries_);
      battery_manager_->resetTimestamp();
    }
    yield();
  }
}

}}  // namespace hyped::sensors
