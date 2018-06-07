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
   *
   * @param[in]    The line format of the input file would be the following
   *
   *               acc_timestamp acc_value_x acc_value_y acc_value_z
   *                             acc_noise_x acc_noise_y acc_noise_z
   *               gyr_timestamp gyr_value_x gyr_value_y gyr_value_z
   *                             gyr_noise_x gyr_noise_y gyr_noise_z
   *
   *               Sample of the format is located at 'src/fake_imu_input.txt'
   */
  explicit FakeImu(std::string acc_file_path, std::string gyr_file_path);

  /*
   * @brief     A constructor for the fake IMU class
   */
  explicit FakeImu(NavigationVector acc_val, NavigationVector acc_noise,
                   NavigationVector gyr_val, NavigationVector gyr_noise);

  /*
   * @brief     A function that gets the imu data
   */
  void getData(Imu* imu) override;

 private:
  const double kAccTimeInterval = 0.000250;
  const double kGyrTimeInterval = 0.000125;

  /*
   * @brief     A function that initializes the variables
   */
  void setData();

  /*
   * @brief     A function that reads data from file directory
   *
   * @param[in]    The file format is as stated in the constructor comments
   */
  void readDataFromFile(std::string acc_file_path, std::string gyr_file_path);

  /*
   * @brief     A function that adds noise to the imu data
   */
  static NavigationVector addNoiseToData(NavigationVector value, NavigationVector noise);

  /*
   * @brief     Checks to see if sufficient time has pass for the sensor to be updated
   */
  bool accCheckTime();
  bool gyrCheckTime();

  bool read_file;

  NavigationVector acc_val, gyr_val;
  NavigationVector acc_noise, gyr_noise;

  DataPoint<NavigationVector> prev_acc;
  DataPoint<NavigationVector> prev_gyr;

  unsigned pt_acc, pt_gyr;
  std::vector<DataPoint<NavigationVector>> acc_val_read;
  std::vector<DataPoint<NavigationVector>> gyr_val_read;

  unsigned acc_count, gyr_count;
  high_resolution_clock::time_point imu_ref_time;
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_
