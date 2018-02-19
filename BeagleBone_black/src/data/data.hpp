/*
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: Class for data exchange between sub-team threads and structures for holding data
 *              produced by each of the sub-teams.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_DATA_DATA_HPP_
#define BEAGLEBONE_BLACK_DATA_DATA_HPP_

#include <cstdint>

namespace hyped {
namespace data {

// -----------------------------------------------------------------------------
// Navigation
// -----------------------------------------------------------------------------
struct Navigation {
  uint32_t distance;
  uint32_t velocity;
  int32_t acceleration;
  uint64_t stripe_count;
};

// -----------------------------------------------------------------------------
// Raw Sensor data
// -----------------------------------------------------------------------------
#define IMU_NUM   3
#define PROXY_NUM 3

struct IMU {
  uint8_t acc_x;
  uint8_t acc_y;
  uint8_t acc_z;

  uint8_t gyr_x;
  uint8_t gyr_y;
  uint8_t gyr_z;
};

struct Proxy {
  uint8_t val;
};

struct Sensors {
  IMU     imu[IMU_NUM];
  Proxy   proxy[PROXY_NUM];
};


// -----------------------------------------------------------------------------
// Common Data structure/class
// -----------------------------------------------------------------------------
/**
 * @brief      A singleton class managing the data exchange between sub-team threads.
 */
class Data {
 public:
  /**
   * @brief      Always returns a reference to the only instance of `Data`.
   */
  static Data& getInstance();

  /**
   * @brief      Retrieves data produced by navigation sub-team.
   */
  Navigation getNavigationData() const;
  /**
   * @brief      Should be called by navigation sub-team whenever they have new data.
   */
  void setNavigationData(const Navigation& nav_data);

  /**
   * @brief      Retrieves data from all sensors
   */
  Sensors getSensorsData() const;

  /**
   * @brief      Should be called to update sensor data
   */
  void setSensorsData(const Sensors& sensors_data);

 private:
  Navigation  navigation_;
  Sensors     sensors_;
};

}}  // namespace hyped::data

#endif  // BEAGLEBONE_BLACK_DATA_DATA_HPP_
