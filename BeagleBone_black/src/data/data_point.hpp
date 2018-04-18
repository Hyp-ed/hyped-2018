/*
 * Author: Branislav Pilnan
 * Organisation: HYPED
 * Date: 04/03/2018
 * Description: Class for time-stamping data
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

#ifndef BEAGLEBONE_BLACK_DATA_DATA_POINT_HPP_
#define BEAGLEBONE_BLACK_DATA_DATA_POINT_HPP_

#include <cstdint>

namespace hyped {
namespace data {

template <typename T>
class DataPoint
{
 public:
  DataPoint()
  {}

  /*
   * Initialises this DataPoint with the specified timestamp and value.
   */
  DataPoint(uint32_t timestamp, const T& value) : timestamp(timestamp), value(value)
  {}

  uint32_t timestamp;
  T value;
};

}}  // namespace hyped::data

#endif  // BEAGLEBONE_BLACK_DATA_DATA_POINT_HPP_
