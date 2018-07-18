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
                       ImuManager::DataArray *imu)
    : ImuManagerInterface(log),
      sys_(System::getSystem()),
      sensors_imu_(imu),
      chip_select_ {48, 49, 115, 117},
      is_calibrated_(false),
      calib_counter_(0)
{
  old_timestamp_ = utils::Timer::getTimeMicros();

  if (sys_.fake_imu || sys_.fake_sensors) is_fake_ = true;

  if (!is_fake_) {
    // create IMUs
    utils::io::SPI::getInstance().setClock(utils::io::SPI::Clock::k1MHz);
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i] = new MPU9250(log, chip_select_[i], 0x08, 0x00);
    }
    utils::io::SPI::getInstance().setClock(utils::io::SPI::Clock::k20MHz);
  } else {
    if (sys_.fail_acc_imu) {
      for (int i = 0; i < data::Sensors::kNumImus - 1; i++) {
        imu_[i] = new FakeImu(log,
                "../BeagleBone_black/data/in/fake_imu_acc_offline_suddenly.txt",
                "../BeagleBone_black/data/in/fake_imu_input_dec.txt",
                "../BeagleBone_black/data/in/fake_imu_input_em.txt",
                "../BeagleBone_black/data/in/fake_imu_input_gyr.txt");
      }
      imu_[data::Sensors::kNumImus-1] = new FakeImu(log,
                  "../BeagleBone_black/data/in/fake_imu_input_acc.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_dec.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_em.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_gyr.txt");
    } else if (sys_.fail_dec_imu) {
      for (int i = 0; i < data::Sensors::kNumImus - 1; i++) {
        imu_[i] = new FakeImu(log,
                "../BeagleBone_black/data/in/fake_imu_input_acc.txt",
                "../BeagleBone_black/data/in/fake_imu_dec_offline_suddenly.txt",
                "../BeagleBone_black/data/in/fake_imu_input_em.txt",
                "../BeagleBone_black/data/in/fake_imu_input_gyr.txt");
      }
      imu_[data::Sensors::kNumImus-1] = new FakeImu(log,
                  "../BeagleBone_black/data/in/fake_imu_input_acc.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_dec.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_em.txt",
                  "../BeagleBone_black/data/in/fake_imu_input_gyr.txt");
    } else {
      // Nominal fake IMUs
      for (int i = 0; i < data::Sensors::kNumImus; i++) {
        if (sys_.accurate) {
          imu_[i] = new FakeAccurateImu(log);
        } else {
          imu_[i] = new FakeImu(log,
                    "../BeagleBone_black/data/in/fake_imu_input_acc.txt",
                    "../BeagleBone_black/data/in/fake_imu_input_dec.txt",
                    "../BeagleBone_black/data/in/fake_imu_input_em.txt",
                    "../BeagleBone_black/data/in/fake_imu_input_gyr.txt");
        }
      }
    }
  }
}

void ImuManager::run()
{
  // collect calibration data
  while (!is_calibrated_) {
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      Imu imu;
      imu_[i]->getData(&imu);
      if (imu.operational) {
        stats_[i][0].update(imu.acc);
        stats_[i][1].update(imu.gyr);
      }
    }
    calib_counter_++;
    if (calib_counter_ >= 100) is_calibrated_ = true;
  }
  log_.INFO("IMU-MANAGER", "Calibration complete!");

  // collect real data
  while (1) {
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imu_[i]->getData(&(sensors_imu_->value[i]));
    }
    if (is_fake_) Thread::sleep(20);
    sensors_imu_->timestamp = utils::Timer::getTimeMicros();
  }
}

ImuManager::CalibrationArray ImuManager::getCalibrationData()
{
  while (!is_calibrated_) {
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
