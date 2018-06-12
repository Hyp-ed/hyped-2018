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

/*
 * @brief    This class is to imitate an IMU. This works by calling the constructor once
 *           and calling getData function multiple times at different time periods to produce
 *           reading that will be used by other classes.
 */
class FakeImu : public ImuInterface {
 public:
  /*
   * @brief     A constructor for the fake IMU class by reading from file
   *
   * @param[in]    The line format of the input file would be the following
   *
   *               timestamp value_x value_y value_z noise_x noise_y noise_z
   *
   *               Sample of the format is located at 'src/fake_imu_input_xxx.txt'. Note that the
   *               timestamp for accelerometer has to start with 0 and must be multiples of 250 and
   *               125 for accelerometer and gyroscope respectively. You must include every timestamp
   *               from 0 to the last timestamp which will be a multiple of 250 or 125 depending on
   *               if it is an accelerometer or gyroscope.
   *
   * @param[in] acc_file_path    A string to the file location of the accelerometer data points
   * @param[in] gyr_file_path    A string to the file location of the gyroscope data points
   */
  explicit FakeImu(std::string acc_file_path, std::string gyr_file_path);

  /*
   * @brief     A constructor for the fake IMU class. This works by generating random numbers
   *            using a normal distribution with xxx_val as mean and xxx_noise as standard deviation.
   */
  explicit FakeImu(NavigationVector acc_val, NavigationVector acc_noise,
                   NavigationVector gyr_val, NavigationVector gyr_noise);

  /*
   * @brief     A function that gets the imu data at the time of call. The function will return
   *            the same data point if the time period since the last update isn't long enough. It
   *            will also skip a couple of data points if the time since the last call has been
   *            sufficiently long.
   */
  void getData(Imu* imu) override;

 private:
  const uint64_t kAccTimeInterval = 250;
  const uint64_t kGyrTimeInterval = 125;

  /*
   * @brief     A function that initializes the common variables
   */
  void setData();

  /*
   * @brief     A function that reads data from file directory
   *
   * @param[in]    The file format is as stated in the constructor comments
   */
  void readDataFromFile(std::string acc_file_path, std::string gyr_file_path);

  /*
   * @brief     A function that adds noise to the imu data using normal distribution
   *
   * @param[in] value    This is the mean of the normal distribution
   * @param[in] noise    This is the standard deviation of the normal distribution
   *
   * @return    Returns random data point value
   */
  static NavigationVector addNoiseToData(NavigationVector value, NavigationVector noise);

  /*
   * @brief     Checks to see if sufficient time has pass for the sensor to be updated and checks if
   *            some data points need to be skipped
   */
  bool accCheckTime();
  bool gyrCheckTime();

  /*
   * @brief     This function takes in the file path of both the files and validates them by checking if
   *              1) The timestamp values are valid. Multiples of 250 or 150 depending on the file.
   *              2) The file follows the format given in the comments of the constructor above.
   *              3) The file exists.
   */
  bool checkFile(std::string acc_file_path, std::string gyr_file_path);

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
