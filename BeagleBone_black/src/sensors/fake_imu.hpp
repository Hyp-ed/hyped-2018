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
#include "sensors/interface.hpp"
#include "utils/logger.hpp"

namespace hyped {

using data::Imu;
using data::DataPoint;
using data::NavigationType;
using data::NavigationVector;

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
  FakeImu(utils::Logger& log_,
          std::string acc_file_path,
          std::string dec_file_path,
          std::string em_file_path,
          std::string gyr_file_path);

  bool isOnline() override { return true; }
  /*
   * @brief     A function that gets the imu data at the time of call. The function will return
   *            the same data point if the time period since the last update isn't long enough. It
   *            will also skip a couple of data points if the time since the last call has been
   *            sufficiently long.
   */
  void getData(Imu* imu) override;

 private:
  utils::Logger&       log_;
  const uint64_t kAccTimeInterval = 50;
  const uint64_t kGyrTimeInterval = 50;
  void startAcc();
  void startDec();
  void startEm();

  /*
   * @brief     A function that reads data from file directory. This function also validates them
   *            by checking if
   *              1) The timestamp values are valid. Multiples of 250 or 150 depending on the file.
   *              2) The file follows the format given in the comments of the constructor above.
   *              3) The file exists.
   *
   * @param[in]    The file format is as stated in the constructor comments
   */
  void readDataFromFile(std::string acc_file_path,
                        std::string dec_file_path,
                        std::string em_file_path,
                        std::string gyr_file_path);

  /*
   * @brief     A function that adds noise to the imu data using normal distribution
   *
   * @param[in] value    This is the mean of the normal distribution
   * @param[in] noise    This is the standard deviation of the normal distribution
   *
   * @return    Returns random data point value
   */
 public:
  static NavigationVector addNoiseToData(NavigationVector value, NavigationVector noise);

 private:
  /*
   * @brief     Checks to see if sufficient time has pass for the sensor to be updated and checks if
   *            some data points need to be skipped
   */
  bool accCheckTime();
  bool gyrCheckTime();

  NavigationVector acc_val_, gyr_val_;
  NavigationVector acc_noise_, gyr_noise_;

  NavigationVector prev_acc_;
  NavigationVector prev_gyr_;

  std::vector<NavigationVector> acc_val_read_;
  std::vector<bool>             acc_val_operational_;
  std::vector<NavigationVector> dec_val_read_;
  std::vector<bool>             dec_val_operational_;
  std::vector<NavigationVector> gyr_val_read_;
  std::vector<bool>             gyr_val_operational_;
  std::vector<NavigationVector> em_val_read_;
  std::vector<bool>             em_val_operational_;

  int64_t acc_count_, gyr_count_;
  uint64_t imu_ref_time_;
  std::string acc_file_path_;
  std::string gyr_file_path_;
  std::string dec_file_path_;
  std::string em_file_path_;
  bool acc_started_;
  bool dec_started_;
  bool em_started_;
  data::Data&  data_;
};

class FakeAccurateImu: public ImuInterface {
 public:
  explicit FakeAccurateImu(utils::Logger& log_);

  bool isOnline() override { return true; }
  void getData(Imu* imu) override;

 private:
  data::Data&    data_;
  NavigationVector acc_noise_, gyr_noise_;
  utils::Logger& log_;
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_FAKE_IMU_HPP_
