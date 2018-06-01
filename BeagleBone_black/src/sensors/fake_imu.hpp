/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 28/05/18
 * Description: Main class for fake IMUs
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

#ifndef BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_
#define BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_

#include <string>
#include "data/data.hpp"
#include "sensors/imu_interface.hpp"

namespace hyped {

using data::Imu;
using data::NavigationType;
using data::NavigationVector;

namespace sensors {

class FakeImu : public ImuInterface {
 public:
  /*
   * @brief     A constructor for the fake IMU class
   */
  explicit FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                   NavigationVector gyr_val, NavigationType gyr_noise);

  /*
   * @brief     A function that gets the imu data
   */
  void getData(Imu* imu) override;

  /*
   * @brief     A function that sets the imu data
   */
  void setData();

  /*
   * @brief     A function that reads data from file directory
   */
  void readDataFromFile(std::string file_path);

 private:
  Imu imu_;
  NavigationVector acc_val;
  NavigationVector gyr_val;
  NavigationType acc_noise;
  NavigationType gyr_noise;

  /*
   * @brief     A function that adds noise to the imu data
   */
  static NavigationVector addNoiseToData(NavigationVector value, NavigationType noise);
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_
