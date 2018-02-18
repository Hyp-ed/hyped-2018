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

struct Navigation {
  uint32_t distance;
  uint32_t velocity;
  int32_t acceleration;
  uint64_t stripe_count;
};

/**
 * @brief      Collates sub-team data structs
 */
struct DataStruct {
  Navigation navigation;
};

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

 private:
  DataStruct data_;
};

}}  // namespace hyped::data

#endif  // BEAGLEBONE_BLACK_DATA_DATA_HPP_
