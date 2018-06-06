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
#include <vector>

#include "data/data.hpp"
#include "sensors/imu_interface.hpp"

namespace hyped {

using data::Imu;
using data::DataPoint;
using data::NavigationType;
using data::NavigationVector;
using std::chrono::high_resolution_clock;

namespace sensors {

class FakeImu : public ImuInterface {
 public:
  /*
   * @brief     A constructor for the fake IMU class by reading from file
   */
  explicit FakeImu(std::string file_path);

  /*
   * @brief     A constructor for the fake IMU class
   */
  explicit FakeImu(NavigationVector acc_val, NavigationType acc_noise,
                   NavigationVector gyr_val, NavigationType gyr_noise);

  /*
   * @brief     Initializes the values of some variables accordingly
   */
  void init();

  /*
   * @brief     A function that gets the imu data
   */
  void getData(Imu* imu) override;

  /*
   * @brief     A function that sets the imu data
   */
  void setData();

 private:
  Imu imu_;

  NavigationVector acc_val;
  NavigationVector gyr_val;
  std::vector<NavigationVector> acc_val_read;
  std::vector<NavigationVector> gyr_val_read;

  NavigationType acc_noise;
  NavigationType gyr_noise;
  std::vector<NavigationType> acc_noise_read;
  std::vector<NavigationType> gyr_noise_read;

  high_resolution_clock::time_point accPrevReadTime;
  high_resolution_clock::time_point gyrPrevReadTime;

  DataPoint<NavigationVector> prevAccData;
  DataPoint<NavigationVector> prevGyrData;

  /*
   * @brief     A function that reads data from file directory
   *
   * @param[in] file_path    The file is in the format acc_val acc_noise gyr_val gyr_noise
   *
   * @return     Returns true if the file was successfully read
   */
  void readDataFromFile(std::string file_path);

  /*
   * @brief     A function that adds noise to the imu data
   */
  static NavigationVector addNoiseToData(NavigationVector value, NavigationType noise);

  /*
   * @brief     Checks to see if sufficient time has pass for the sensor to be updated
   */
  bool accCheckTime();
  bool gyrCheckTime();
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_
