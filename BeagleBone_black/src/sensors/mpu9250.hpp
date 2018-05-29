/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 23/05/18
 * Description: Main file for MPU9250
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

#ifndef BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_
#define BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_

#include <vector>

#include "sensors/imu_interface.hpp"
#include "utils/logger.hpp"
#include "utils/io/spi.hpp"
#include "utils/io/gpio.hpp"

namespace hyped {

using hyped::utils::io::SPI;
using utils::Logger;
using utils::io::GPIO;


namespace sensors {

class MPU9250 : ImuInterface {
 public:
  // TODO(Jack)
  explicit MPU9250(Logger& log, uint32_t pin);
  ~MPU9250();

  void getData(Imu* imu) override;

 private:
  /*
   *  @brief Calibrates the accelerometer
   */
  void calibrateAccl();
  /*
   *  @brief Sets the range for the gyroscope
   */
  void setGyroScale(int scale);
  /*
   *  @brief Sets the range for the accelerometer
   */
  void setAcclScale(int scale);
  /*
   *  @brief Returns the most recent Accelerometer readings
   *
   *  @return 3Dvector Returns accelerometer readings
   */
  double getAcclData();
  /*
   *  @brief Returns the most recent gyroscope readings
   *
   *  @return 3Dvector Returns gyroscope readings
   */
  double getGyroData();
  /*
   *  @brief Performs the readings of gyroscope & accelerometer
   */
  void performSensorReadings();

 private:
  static const uint64_t time_start;
  // TODO(Jack)
  // Call this function at startup
  void init();
  void select();
  void deSelect();
  bool whoAmI();
  // Write to a register
  bool writeByte(uint8_t write_reg, uint8_t *write_data);
  // Read from a register
  bool readByte(uint8_t read_reg, uint8_t *read_data);
  SPI& spi_ = SPI::getInstance();
  GPIO gpio_;
  double accl_scale_;
  double gyro_scale_;
  uint8_t cs_;
  Logger& log_;
};

}}  // namespace hyped::sensors


#endif  // BEAGLEBONE_BLACK_SENSORS_MPU9250_HPP_
