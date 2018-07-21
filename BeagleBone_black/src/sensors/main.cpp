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
#ifdef PROXI
#include "sensors/proxi_manager.hpp"
#endif
#include "sensors/fake_gpio_counter.hpp"
#include "sensors/gpio_counter.hpp"
#include "sensors/em_brake.hpp"

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
#ifdef PROXI
      proxi_manager_front_(new ProxiManager(log, true, &sensors_.proxi_front)),
      proxi_manager_back_(new ProxiManager(log, false, &sensors_.proxi_back)),
#endif
      battery_manager_(new BmsManager(log,
                                         &batteries_.low_power_batteries,
                                         &batteries_.high_power_batteries)),
      sensor_init_(false),
      battery_init_(false)
{
  // @TODO (Anyone) Check THESE PINS
  if (sys_.fake_sensors || sys_.fake_keyence) {
    keyence_l_ = new FakeGpioCounter(log, sys_.miss_keyence, sys_.double_keyence);
    keyence_r_ = new FakeGpioCounter(log, sys_.miss_keyence, sys_.double_keyence);
    optical_encoder_l_ = new FakeGpioCounter(log, false, false);
    optical_encoder_r_ = new FakeGpioCounter(log, false, false);
  } else {
    // Pins for keyence GPIO_73 and GPIO_75
    GpioCounter* temp;
    temp = new GpioCounter(66);
    temp->start();
    keyence_l_ = temp;

    temp = new GpioCounter(67);
    temp->start();
    keyence_r_ = temp;

    temp = new GpioCounter(69);
    temp->start();
    optical_encoder_l_ = temp;

    temp = new GpioCounter(68);
    temp->start();
    optical_encoder_r_ = temp;
  }
  EmBrake* em_brake_front_ = new EmBrake(log, true);
  EmBrake* em_brake_rear_ = new EmBrake(log, false);
  em_brake_front_->start();
  em_brake_rear_->start();
}

void Main::run()
{
  // start all managers
  imu_manager_->start();
#ifdef PROXI
  proxi_manager_front_->start();
  proxi_manager_back_->start();
#endif
  battery_manager_->start();

  // init loop
  while (!sensor_init_) {
    if (imu_manager_->updated()
#ifdef PROXI
    && proxi_manager_front_->updated() && proxi_manager_back_->updated()
#endif
      ) {
      data_.setSensorsData(sensors_);

      // Get calibration data
      SensorCalibration sensor_calibration_data;
#ifdef PROXI
      sensor_calibration_data.proxi_front_variance = proxi_manager_front_->getCalibrationData();
      sensor_calibration_data.proxi_back_variance  = proxi_manager_back_->getCalibrationData();
#endif
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
      data_.setBatteryData(batteries_);
      battery_init_ = true;
      break;
    }
    yield();
  }
  log_.INFO("SENSORS", "batteries data has been initialised");

  if (sensor_init_) sensors_.module_status = data::ModuleStatus::kInit;
  if (battery_init_) batteries_.module_status = data::ModuleStatus::kInit;

  // work loop
  while (sys_.running_) {
    // Write sensor data to data structure only when all the imu or proxi values are different
    if (imu_manager_->updated()) {
      sensors_.keyence_stripe_counter[0] = keyence_l_->getStripeCounter();
      sensors_.keyence_stripe_counter[1] = keyence_r_->getStripeCounter();
      sensors_.optical_enc_distance[0] = optical_encoder_l_->getStripeCounter().count.value *
                                      M_PI *
                                      kWheelDiameter;
      sensors_.optical_enc_distance[1] = optical_encoder_r_->getStripeCounter().count.value *
                                      M_PI *
                                      kWheelDiameter;
      data_.setSensorsData(sensors_);
      // Update manager timestamp with a function
      imu_manager_->resetTimestamp();
    }
#ifdef PROXI
    if (proxi_manager_front_->updated() && proxi_manager_back_->updated()) {
      data_.setSensorsData(sensors_);
      proxi_manager_front_->resetTimestamp();
      proxi_manager_back_->resetTimestamp();
    }
#endif
    // Update battery data only when there is some change
    if (battery_manager_->updated()) {
      battery_manager_->resetTimestamp();

      // check health of batteries
      if (batteries_.module_status != data::ModuleStatus::kCriticalFailure && !sys_.fake_sensors) {
        if (!batteriesInRange()) {
          log_.ERR("SENSORS", "battery failure detected");
          batteries_.module_status = data::ModuleStatus::kCriticalFailure;
        }
      }

      // publish the new data
      data_.setBatteryData(batteries_);
    }
    yield();
  }
}

bool Main::batteriesInRange()
{
  // check all LP and HP battery values are in expected range

  // check LP
  for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
    auto& battery = batteries_.low_power_batteries[i];
    if (battery.voltage < 140 || battery.voltage > 252) {   // voltage in 14V to 25.2V
      log_.ERR("SENSORS", "BMS LP %d voltage out of range: %d", i, battery.voltage);
      return false;
    }

    if (battery.current < 0 || battery.current > 300) {       // current in 0A to 30A
      log_.ERR("SENSORS", "BMS LP %d current out of range: %d", i, battery.current);
      return false;
    }

    if (battery.temperature < -20 || battery.temperature > 70) {  // temperature in -20C to 70C
      log_.ERR("SENSORS", "BMS LP %d temperature out of range: %d", i, battery.temperature);
      return false;
    }
  }

  // check HP
  for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
    auto& battery = batteries_.high_power_batteries[i];
    if (battery.voltage < 720 || battery.voltage > 1246) {   // voltage in 72V to 124.6V
      log_.ERR("SENSORS", "BMS HP %d voltage out of range: %d", i, battery.voltage);
      return false;
    }

    if (battery.current < -4000 || battery.current > 13500) {       // current in -400A to 1350A
      log_.ERR("SENSORS", "BMS HP %d current out of range: %d", i, battery.current);
      return false;
    }

    if (battery.temperature < -20 || battery.temperature > 70) {  // temperature in -20C to 70C
      log_.ERR("SENSORS", "BMS HP %d temperature out of range: %d", i, battery.temperature);
      return false;
    }
  }
  return true;
}

}}  // namespace hyped::sensors
