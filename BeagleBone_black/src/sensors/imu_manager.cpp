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


#include "sensors/imu_manager.hpp"

#include "sensors/mpu9250.hpp"
#include "data/data.hpp"
#include "utils/timer.hpp"
#include "sensors/fake_imu.hpp"

namespace hyped {

using data::Data;
using data::Sensors;
using utils::System;
using data::NavigationVector;

namespace sensors {

ImuManager::ImuManager(Logger& log,
                       data::DataPoint<array<Imu, data::Sensors::kNumImus>> *imu)
    : ImuManagerInterface(log),
      sys_(System::getSystem()),
      data_(Data::getInstance()),
      chip_select_ {31, 50, 48, 51}
{
  old_timestamp_ = utils::Timer::getTimeMicros();
  if (sys_.fake_imu || sys_.fake_sensors) is_fake_ = true;
  if (!is_fake_) {
    // create IMUs
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i] = new MPU9250(log, chip_select_[i], 0x08, 0x00);
      imu_calibrations_[i] = imu_[i]->calcCalibrationData();
    }
  } else {
    // create fake IMUs
    NavigationVector acc;
    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 9.8;
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i] = new FakeImuStationary(log,
                                      acc,
                                      NavigationVector(1),
                                      NavigationVector(0),
                                      NavigationVector(1));
      imu_calibrations_[i] = imu_[i]->calcCalibrationData();
      imu_accelerating_[i] = new FakeImuAccelerating(log,
                                                    "../BeagleBone_black/data/in/fake_imu_input_acc.txt",  //NOLINT
                                                    "../BeagleBone_black/data/in/fake_imu_input_gyr.txt"); //NOLINT
      imu_decelerating_[i] = new FakeImuAccelerating(log,
                                                    "../BeagleBone_black/data/in/fake_imu_input_dec.txt",  //NOLINT
                                                    "../BeagleBone_black/data/in/fake_imu_input_gyr.txt"); //NOLINT
      // TODO(anyone) add fake calcCalibrationData()
    }
  }
  sensors_imu_ = imu;
}

void ImuManager::run()
{
  while (1) {
    // If the state changes to accelerating use different data
    if (is_fake_ == true && data_.getStateMachineData().current_state == data::State::kAccelerating) { //NOLINT
      for (int i =0; i < data::Sensors::kNumImus; i++) {
        imu_accelerating_[i]->getData(&(sensors_imu_->value[i]));
      }
    } else if (is_fake_ == true && data_.getStateMachineData().current_state == data::State::kDecelerating) { //NOLINT
      for (int i =0; i < data::Sensors::kNumImus; i++) {
        imu_decelerating_[i]->getData(&(sensors_imu_->value[i]));
      }
    } else {
      for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i]->getData(&(sensors_imu_->value[i]));
      }
    }
    sensors_imu_->timestamp = utils::Timer::getTimeMicros();
  }
}

array<array<NavigationVector, 2>, data::Sensors::kNumImus> ImuManager::getCalibrationData()
{
  return imu_calibrations_;
}

bool ImuManager::updated()
{
  if (old_timestamp_ != sensors_imu_->timestamp) {
    return true;
  }
  return false;
}

void ImuManager::resetTimestamp()
{
  old_timestamp_ = sensors_imu_->timestamp;
}
}}  // namespace hyped::sensors
