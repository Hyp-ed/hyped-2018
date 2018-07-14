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
      chip_select_ {31, 50, 48, 51},
      is_calib_(false),
      calib_counter_(0)
{
  old_timestamp_ = utils::Timer::getTimeMicros();
  if (sys_.fake_imu || sys_.fake_sensors) is_fake_ = true;
  if (!is_fake_) {
    // create IMUs
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i] = new MPU9250(log, chip_select_[i], 0x08, 0x00);
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
  while (!is_calib_) {
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      Imu imu;
      imu_[i]->getData(&imu);
      if (imu.operational) {
        stats_[i][0].update(imu.acc);
        stats_[i][1].update(imu.gyr);
      }
    calib_counter_++;
    if (calib_counter_ >= 100) is_calib_ = true;
    }
  }

  while (1) {
    data::State state_ = data_.getStateMachineData().current_state;
    // If the state changes to accelerating use different data
    if (is_fake_ && state_ == data::State::kAccelerating) { //NOLINT
      for (int i = 0; i < data::Sensors::kNumImus; i++) {
        imu_accelerating_[i]->getData(&(sensors_imu_->value[i]));
      }
    } else if (is_fake_ && (state_ == data::State::kDecelerating || state_ == data::State::kEmergencyBraking)) { //NOLINT
      for (int i = 0; i < data::Sensors::kNumImus; i++) {
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
  while (!is_calib_) {
    Thread::yield();
  }
  for (int i = 0; i < data::Sensors::kNumImus; i++) {
    imu_calibrations_[i][0] = stats_[i][0].getVariance();
    imu_calibrations_[i][1] = stats_[i][1].getVariance();
  }
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
